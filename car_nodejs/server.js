const TF_converter = require('./tf_converter/TF_converter.js');
const rosnodejs = require('rosnodejs');
const actionlib = rosnodejs.require('actionlib');
const express = require('express');
const app = express();
const cors = require('cors');
const port = 3000;
const bodyParser = require('body-parser');
app.use(cors()); // 这里添加CORS中间件
app.use(bodyParser.json());
app.use(express.json());


let currentPose = null;
let navigationStatus;
let joystickStatus = false;
let map_match_ratio;

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

      //-------------------------API-------------------------//
      // API connect check / navigationStatus / joystickStatus
      app.get('/connect', (req, res) => {
         res.status(200).send({
            connect: 'Success Connected to the server.', //connect msg
            navigationStatus: navigationStatus, //navigation status
            joystickStatus: joystickStatus, //joystick status
         });
      });

      // API current_pose
      app.get('/current_pose', (req, res) => {
         if (currentPose) {
            res.status(200).send({
               currentPose: currentPose,
               map_match_ratio: map_match_ratio,
            });
         } else {
            res.status(404).send('No pose information available.');
         }
      });

      //API Relocation_initpose
      app.post('/relocation_initpose', express.json(), (req, res) => {
         const value = req.body.relocation_initpose_value;
         console.log('Received relocation_initpose value:', value);
         // check if a value was provided and if it is a number
         if (value !== undefined && !isNaN(value)) {
            let msg = new std_msgs.Int16();
            msg.data = value;
            console.log(msg);
            initialize_Relocation_pub.publish(msg);
            res.status(200).send('Relocation_initpose Message published successfully.');
         } else {
            res.status(400).send('No valid number provided.');
         }
      });

      //API Path Cancel
      app.post('/path_cancel', express.json(), (req, res) => {
         const path_cancel_value = req.body.path_cancel_value;
         console.log('Received path_cancel value:', path_cancel_value);
         // check if a path_cancel_value was provided and if it is a number
         if (path_cancel_value !== undefined && !isNaN(path_cancel_value)) {
            let msg = new std_msgs.Int8();
            msg.data = path_cancel_value;
            stopnavigation_pub.publish(msg);
            res.status(200).send('Path Cancel Message published successfully.');
         } else {
            res.status(400).send('No valid number provided.');
         }
      });

      //API set_goal
      app.post('/set_goal', express.json(), (req, res) => {
         const multiple_points = req.body.multiple_points;
         console.log('Received set_goal value:', multiple_points);
         let goals = [];

         if (multiple_points) {
            multiple_points.forEach(point => {
               let goalMsg = new geometry_msgs.Pose();

               // 假设point包含x, y, 和 yaw
               goalMsg.position.x = point.x;
               goalMsg.position.y = point.y;
               goalMsg.position.z = 0;  // 如果在二维平面上，通常Z是0

               // 假设yaw是以弧度表示的方向，将其转换为四元数
               let quaternion = TF_converter(point.yaw, 0, 0);
               goalMsg.orientation = quaternion;

               // 确保所有值都正确赋给goalMsg
               //  console.log(goalMsg);

               goals.push(goalMsg);
            });

            // 构造PoseArray消息
            const PoseArray = rosnodejs.require('geometry_msgs').msg.PoseArray;
            let poseArrayMsg = new PoseArray();
            poseArrayMsg.poses = goals;

            // 发布PoseArray消息
            multiple_goals_pub.publish(poseArrayMsg);

            //   console.log('Published goals:', goals);
            res.status(200).send('Goals published successfully.');
         } else {
            res.status(400).send('Invalid parameters provided.');
         }
      });

      //API Go_home
      app.post('/go_home', express.json(), (req, res) => {
         const go_home_value = req.body.go_home_value;
         console.log('Received go_home value:', go_home_value);

         // 确保收到了所有必要的坐标和角度值
         if (go_home_value !== undefined && go_home_value === 40) {
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

            res.status(200).send('Go Home Msg Send successfully.');
         } else {
            res.status(400).send('Invalid parameters provided.');
         }
      });

      // 監聽指定端口
      app.listen(port, () => {
         console.log(`Server running on port:${port}`);
      });
   })
   .catch((error) => {
      console.error('ROS Node failed to initialize', error);
   });
