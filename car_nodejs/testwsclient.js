const TF_converter = require('./tf_converter/TF_converter.js');
const rosnodejs = require('rosnodejs');
const express = require('express');
const WebSocket = require('ws');
const cors = require('cors');

const app = express();
const wsUrl = 'ws://192.168.103.83:9090';
const port = 3009;
const sendInterval = 1000; // 1 second
const reconnectInterval = 3000; // 重新連接的間隔（毫秒）
const bodyParser = require('body-parser');
app.use(cors()); // 这里添加CORS中间件
app.use(bodyParser.json());
app.use(express.json());


let currentPose = null;
let navigationStatus;
let joystickStatus = false;
let map_match_ratio;
let receivedDatastruct = {
    relocation_initpose_value: null,
    relocation_initpose_bool: false,
    path_cancel_value: null,
    path_cancel_bool: false,
    go_home_value: null,
    go_home_bool: false,
    multiple_points: [],
    multiple_points_bool: false,
};

// 初始化ROS節點
rosnodejs.initNode('/my_ros_node')
    .then((rosNode) => {

        // ---------------subscribe-------------------//

        const nh = rosnodejs.nh;
        nh.subscribe('/amcl_pose', 'geometry_msgs/PoseWithCovarianceStamped', (msg) => {
            currentPose = msg.pose.pose; // 儲存當前位置
        });
        nh.subscribe('/navigation_status', 'std_msgs/String', (msg) => {
            navigationStatus = msg.data;
        });

        nh.subscribe('/joy_enable', 'std_msgs/Bool', (msg) => {
            joystickStatus = msg.data;
        });

        nh.subscribe('/map_match_ratio', 'std_msgs/Float32', (msg) => {
            map_match_ratio = msg.data;
        });

        // ---------------subscribe-------------------//


        // ---------------publish---------------//

        const std_msgs = rosnodejs.require('std_msgs').msg;
        let initialize_Relocation_pub = nh.advertise('/initialize_Relocation', std_msgs.Int16);
        let stopnavigation_pub = nh.advertise("/stopnavigationSend", std_msgs.Int8);

        const geometry_msgs = rosnodejs.require('geometry_msgs').msg;
        let goalPub = nh.advertise('/move_base_simple/goal', geometry_msgs.PoseStamped);
        let multiple_goals_pub = nh.advertise('/multiple_goals', geometry_msgs.PoseArray);

        // ---------------publish---------------//

        // 中间件，允许跨域请求 (CORS)
        app.use((req, res, next) => {
            res.header('Access-Control-Allow-Origin', '*');
            res.header('Access-Control-Allow-Headers', 'Origin, X-Requested-With, Content-Type, Accept');
            next();
        });

        //---------------------websocket----------------------//

        function connectWebSocket() {
            const ws = new WebSocket(wsUrl);

            ws.on('open', function open() {
                console.log('Connected to WebSocket server');
                
                // 定期發送資訊
                setInterval(() => {
                    ws.send(JSON.stringify({
                        // clientId: 0x01,
                        connect: 'Success Connected to the server.',
                        navigationStatus: navigationStatus,
                        joystickStatus: joystickStatus,
                        currentPose: currentPose,
                        map_match_ratio: map_match_ratio,
                    }));
                }, sendInterval);
                ws.send(JSON.stringify({ clientId: 0x02 }));
            });

            ws.on('message', function incoming(message) {
                try {
                    const receivedData = JSON.parse(message);
                    receivedDatastruct = receivedData;
                    console.log('Received Server Data:', receivedDatastruct);
                    // 進一步處理receivedData

                    //Relocation publish to ROS//
                    if (receivedDatastruct.relocation_initpose_bool) {
                        let msg = new std_msgs.Int16();
                        msg.data = receivedDatastruct.relocation_initpose_value;
                        console.log(msg);
                        initialize_Relocation_pub.publish(msg);
                        receivedDatastruct.relocation_initpose_bool = false;
                    }

                    //Path Cancel publish to ROS//
                    if (receivedDatastruct.path_cancel_bool == true) {
                        let msg = new std_msgs.Int8();
                        msg.data = receivedDatastruct.path_cancel_value;
                        stopnavigation_pub.publish(msg);
                        receivedDatastruct.path_cancel_bool = false;
                    }

                    //Go Home publish to ROS//
                    if (receivedDatastruct.go_home_bool == true) {
                        // 新的PoseStamped消息
                        let goalMsg = new geometry_msgs.PoseStamped();

                        // 设置header
                        goalMsg.header.stamp = rosnodejs.Time.now();
                        goalMsg.header.frame_id = 'map'; // 或者是任何合适的参考框架

                        // 设置位置
                        goalMsg.pose.position.x = 0;
                        goalMsg.pose.position.y = 0;
                        goalMsg.pose.position.z = 0;  // 在2D导航中，通常Z值为0

                        // 设置姿态
                        goalMsg.pose.orientation.x = 0;
                        goalMsg.pose.orientation.y = 0;
                        goalMsg.pose.orientation.z = 0;
                        goalMsg.pose.orientation.w = 1;

                        // 发布目标点
                        goalPub.publish(goalMsg);
                        receivedDatastruct.go_home_bool = false;
                    }
                    //Multiple Goals publish to ROS//
                    if (receivedDatastruct.multiple_points_bool == true) {
                        let goals = [];
                        receivedDatastruct.multiple_points.forEach(point => {
                            let goalMsg = new geometry_msgs.Pose();

                            // 假设point包含x, y, 和 yaw
                            goalMsg.position.x = point.x;
                            goalMsg.position.y = point.y;
                            goalMsg.position.z = 0;  // 如果在二维平面上，通常Z是0

                            // 假设yaw是以弧度表示的方向，将其转换为四元数
                            let quaternion = TF_converter(point.yaw, 0, 0);
                            goalMsg.orientation = quaternion;
                            goals.push(goalMsg);
                        });

                        // build PoseArray消息
                        const PoseArray = rosnodejs.require('geometry_msgs').msg.PoseArray;
                        let poseArrayMsg = new PoseArray();
                        poseArrayMsg.poses = goals;

                        // publish PoseArray消息
                        multiple_goals_pub.publish(poseArrayMsg);

                        //   console.log('Published goals:', goals);
                        receivedDatastruct.multiple_points_bool = false;
                    }
                } catch (error) {
                    console.log('Error parsing JSON:', error);
                }
            });

            ws.on('close', function close() {
                console.log('WebSocket connection closed. Attempting to reconnect...');
                setTimeout(connectWebSocket, reconnectInterval);
            });

            ws.on('error', function error(error) {
                console.error('WebSocket error:', error);
                ws.close(); // 確保在錯誤後關閉連接
            });
        }

        // 初始化連接
        connectWebSocket();

        // 監聽指定端口
        app.listen(port, () => {
            console.log(`Server running on port:${port}`);
        });
    })
    .catch((error) => {
        console.error('ROS Node failed to initialize', error);
    });
