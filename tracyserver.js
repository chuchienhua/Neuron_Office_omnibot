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


// 初始化ROS節點
rosnodejs.initNode('/my_ros_node')
   .then((rosNode) => {
      const nh = rosnodejs.nh;

      // ---------------subscribe-------------------//

      // ---------------subscribe-------------------//


      // ---------------publish---------------//

      const std_msgs = rosnodejs.require('std_msgs').msg;
      let chatgpt_ask_pub = nh.advertise("/chatgpt_ask", std_msgs.String);


      // ---------------publish---------------//

      // 中间件，允许跨域请求 (CORS)
      app.use((req, res, next) => {
         res.header('Access-Control-Allow-Origin', '*');
         res.header('Access-Control-Allow-Headers', 'Origin, X-Requested-With, Content-Type, Accept');
         next();
      });

      //-------------------------API-------------------------//

      //API chatgpt_ask
      app.post('/chatgpt_ask', express.json(), (req, res) => {
        const chatgpt_ask = req.body.chatgpt_ask;
        console.log(chatgpt_ask);
        if (chatgpt_ask !== undefined && !isNaN(chatgpt_ask)) {
          let msg = new std_msgs.String();
          msg.data = chatgpt_ask;
          console.log(msg);
         //  chatgpt_ask_pub.publish(msg);
          
        }
        res.status(200).send('chatgpt_ask Message published successfully.');
      });

      // 監聽指定端口
      app.listen(port, () => {
         console.log(`Server running on port:${port}`);
      });
   })
   .catch((error) => {
      console.error('ROS Node failed to initialize', error);
   });