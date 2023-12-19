const express = require("express");
const rosnodejs = require("rosnodejs");
const http = require("http");
const bodyParser = require('body-parser');
const WebSocket = require("ws");
const cors = require("cors");
const AgvMailRouter = require('./AGV_mail_Router.js')

const app = express();
const port = 9090;
const HEARTBEAT_INTERVAL_TO_client = 20000; // heartbeat 20s
const HEARTBEAT_INTERVAL_Receive_From_webuser = 1000; // heartbeat 20s

app.use(cors());
app.use(express.json());
app.use(bodyParser.json());
let relocation_initpose_bool, path_cancel_bool, go_home_bool, multiple_points_bool, control_mode_bool, chatgpt_msg_bool;
let lastHeartbeatTime = 0;
const HEARTBEAT_TIMEOUT = 5000;

// 創建一個HTTP服務器並將Express應用掛載上去
const server = http.createServer(app);

// 創建WebSocket伺服器並將其掛載到同一個HTTP服務器上
const wss = new WebSocket.Server({ server });
const clientMap = new Map(); // store client id and WebSocket connect
const clientDataMap = new Map(); // store clientID and received data
const heartbeats = new Map(); // erery car ID's heartbeat


wss.on("connection", function connection(ws) {
  console.log('Agv client connected via WebSocket.');
  let clientId = null;

  //setting heartbeat 30s / one time
  const heartbeat = setInterval(() => {
    if (ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({ heartbeat: 0xAA }));
    }
  }, HEARTBEAT_INTERVAL_TO_client);

  ws.on("message", function incoming(message) {
    try {
      const receivedData = JSON.parse(message);
      // 首次接收客户端ID
      if (receivedData.clientId && !clientId) {
        clientId = receivedData.clientId;
        clientMap.set(clientId, ws);
        console.log("Client connected with ID:", clientId);
      }
      if (clientId) {
        clientMap.set(clientId, ws);
        clientDataMap.set(clientId, {
          navigationStatus: receivedData.navigationStatus,
          joystickStatus: receivedData.joystickStatus,
          currentPose: receivedData.currentPose,
          map_match_ratio: receivedData.map_match_ratio,
          startnavigation_status: receivedData.startnavigation_status,
        });
        // console.log(clientId + "Client data updated:", JSON.stringify(clientDataMap.get(clientId)));
      }
      else {
        console.log("Warning: Received message without clientId.");
      }
      //Only not connect keep communication, don't let client disconnect
      if (receivedData.heartbeat === 0XAB) {
        console.log("Heartbeat received from client");
      }
    } catch (error) {
      console.log("Error parsing JSON:", error);
    }
  });
  ws.on("close", () => {
    clientMap.forEach((value, key) => {
      if (value === ws) {
        clientMap.delete(key);
        clientDataMap.delete(key);
        clearInterval(heartbeat);
      };
    });
  });
});

app.use('/AGVMail', AgvMailRouter);

//------cors setting------//
app.use((req, res, next) => {
  res.header("Access-Control-Allow-Origin", "*");
  res.header(
    "Access-Control-Allow-Headers",
    "Origin, X-Requested-With, Content-Type, Accept"
  );
  next();
});

//-------------------------API-------------------------//
// API connect check / navigationStatus / joystickStatus
app.post("/connect", (req, res) => {
  const targetClientId = req.body.carid;
  const connect_bool = req.body.carconnect_bool;
  const connect_heartbeat = req.body.connect_heartbeat;
  const clientData = clientDataMap.get(targetClientId);
  const targetClient = clientMap.get(targetClientId);
  // console.log("connect_heartbeat ", connect_heartbeat);
  // console.log("Received connect value:", connect_bool);

  // wss.clients.forEach((client) => {
  if (targetClient && targetClient.readyState === WebSocket.OPEN) {
    // client.send(
    targetClient.send(
      JSON.stringify({
        connect_bool: connect_bool,
      })
    );
    heartbeats.set(targetClientId, { heartbeat: connect_heartbeat, lastUpdate: Date.now() });
  } else {
    console.log(`No client found with ID ${targetClientId}`);
    res.status(404).send("Client not found");
  }
  // });
  if (clientData) {
    res.status(200).send({
      connect: "Success Connected to the server.", //connect msg
      startnavigation_status: clientData.startnavigation_status, //startnavigation status
    });
  } else {
    res.status(404).send("No pose information available for client " + targetClientId);
    connect_bool = false;
  }
});
/*
check heartbeat 1000ms
*/
setInterval(() => {
  heartbeats.forEach((value, key) => {
    if (Date.now() - value.lastUpdate > 2000) { // 10秒超时
      console.log(`Client ${key} disconnected due to heartbeat timeout.`);

      const targetClient = clientMap.get(key);

      if (targetClient && targetClient.readyState === WebSocket.OPEN) {
        targetClient.send(JSON.stringify({ User_Error_Disconnect: true }));
      }
      heartbeats.delete(key);
    }
  });
}, HEARTBEAT_INTERVAL_Receive_From_webuser);
/*
check heartbeat 1000ms
*/

// API disconnect
app.post("/disconnect", (req, res) => {
  const targetClientId = req.body.carid;
  const connect_bool = req.body.carconnect_bool;
  const targetClient = clientMap.get(targetClientId);
  if (targetClient && targetClient.readyState === WebSocket.OPEN) {
    targetClient.send(
      JSON.stringify({
        connect_bool: connect_bool,
      })
    );
  }
  if (req.body.carid && req.body.carconnect_bool) {
    res.status(200).send({
      disconnect: "Success Disconnected to the server.", //disconnect msg
    });
  }
});

// API current_pose
app.post("/current_pose", (req, res) => {
  const targetClientId = req.body.carid;
  const clientData = clientDataMap.get(targetClientId);
  if (clientData) {
    res.status(200).send({
      currentPose: clientData.currentPose,
      map_match_ratio: clientData.map_match_ratio,
      navigationStatus: clientData.navigationStatus, //navigation status
      joystickStatus: clientData.joystickStatus, //joystick status
    });
  } else {
    res.status(404).send("No pose information available.");
  }
});

// API chatgpt msg
app.post("/chatgpt_msg", (req, res) => {
  const targetClientId = req.body.carid;
  const chatgpt_msg = req.body.chatgpt_msg;
  const targetClient = clientMap.get(targetClientId);
  chatgpt_msg_bool = true;
  if (targetClient && targetClient.readyState === WebSocket.OPEN) {
    targetClient.send(
      JSON.stringify({
        chatgpt_msg_bool: chatgpt_msg_bool,
        chatgpt_msg: chatgpt_msg,
      })
    );
    console.log("Received chatgpt_msg value:", chatgpt_msg);
    res.status(200).send("Chatgpt_MSG Message published successfully.");
  } else {
    chatgpt_msg_bool = false;
    res.status(400).send("No valid deliver chatgpt_msg.");
  }
});

//API Change_contorl_mode
app.post("/control_mode", express.json(), (req, res) => {
  const control_mode_value = req.body.control_mode;
  const targetClientId = req.body.carid;
  const targetClient = clientMap.get(targetClientId);
  console.log("Received relocation_initpose value:", control_mode_value);
  // check if a value was provided and if it is a number
  if (control_mode_value !== undefined && !isNaN(control_mode_value)) {
    control_mode_bool = true;
    if (targetClient && targetClient.readyState === WebSocket.OPEN) {
      // 只向特定客户端发送消息
      targetClient.send(JSON.stringify({
        control_mode_bool: control_mode_bool,
        control_mode_value: control_mode_value,
      }));
    }
    res.status(200).send("Control_mode Message published successfully.");
  } else {
    control_mode_bool = false;
    res.status(400).send("No valid number provided.");
  }
});

//API Relocation_initpose
app.post("/relocation_initpose", express.json(), (req, res) => {
  const value = req.body.relocation_initpose_value;
  const targetClientId = req.body.carid;
  const targetClient = clientMap.get(targetClientId);
  console.log("Received relocation_initpose value:", value);
  // check if a value was provided and if it is a number
  if (value !== undefined && !isNaN(value)) {
    relocation_initpose_bool = true;
    if (targetClient && targetClient.readyState === WebSocket.OPEN) {
      // 只向特定客户端发送消息
      targetClient.send(JSON.stringify({
        relocation_initpose_bool: relocation_initpose_bool,
        relocation_initpose_value: value,
      }));
    }
    res.status(200).send("Relocation_initpose Message published successfully.");
  } else {
    relocation_initpose_bool = false;
    res.status(400).send("No valid number provided.");
  }
});

//API Path Cancel
app.post("/path_cancel", express.json(), (req, res) => {
  const path_cancel_value = req.body.path_cancel_value;
  const targetClientId = req.body.carid;
  const targetClient = clientMap.get(targetClientId);
  console.log("Received path_cancel value:", path_cancel_value);
  // check if a path_cancel_value was provided and if it is a number
  if (path_cancel_value !== undefined && !isNaN(path_cancel_value)) {
    path_cancel_bool = true;
    // wss.clients.forEach((client) => {
    if (targetClient && targetClient.readyState === WebSocket.OPEN) {
      // client.send(
      targetClient.send(
        JSON.stringify({
          path_cancel_bool: path_cancel_bool,
          path_cancel_value: path_cancel_value,
        })
      );
    }
    // });
    res.status(200).send("Path Cancel Message published successfully.");
  } else {
    path_cancel_bool = false;
    res.status(400).send("No valid number provided.");
  }
});

//API set_goal
app.post("/set_goal", express.json(), (req, res) => {
  const multiple_points = req.body.multiple_points;
  const targetClientId = req.body.carid;
  const targetClient = clientMap.get(targetClientId);
  console.log("Received set_goal value:", multiple_points);

  if (multiple_points) {
    multiple_points_bool = true;
    // wss.clients.forEach((client) => {
    if (targetClient && targetClient.readyState === WebSocket.OPEN) {
      // client.send(
      targetClient.send(
        JSON.stringify({
          multiple_points: multiple_points,
          multiple_points_bool: multiple_points_bool,
        })
      );
    }
    // });
    res.status(200).send("Goals published successfully.");
  } else {
    multiple_points_bool = false;
    res.status(400).send("Invalid parameters provided.");
  }
});

//API Go_home
app.post("/go_home", express.json(), (req, res) => {
  const go_home_value = req.body.go_home_value;
  const targetClientId = req.body.carid;
  const targetClient = clientMap.get(targetClientId);
  console.log("Received go_home value:", go_home_value);
  if (go_home_value !== undefined && !isNaN(go_home_value)) {
    go_home_bool = true;
    // wss.clients.forEach((client) => {
    if (targetClient && targetClient.readyState === WebSocket.OPEN) {
      // client.send(
      targetClient.send(
        JSON.stringify({
          go_home_bool: go_home_bool,
          go_home_value: go_home_value,
        })
      );
    }
    // });
    res.status(200).send("Go Home Msg Send successfully.");
  } else {
    go_home_bool = false;
    res.status(400).send("Invalid parameters provided.");
  }
});



// 使用HTTP服務器來監聽端口，而不是直接使用Express應用
server.listen(port, () => {
  console.log(`Server is running on port:${port}`);
});
