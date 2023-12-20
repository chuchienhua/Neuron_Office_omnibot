import memberRouter from "./routes/Mongo_memberApi.js";
import axios from "axios";
import express from "express";
import path from "path";
import logger from "morgan";
import cors from "cors";
import { fileURLToPath } from "url";
import { dirname } from "path";
import http from "http";
import AgvMailRouter from "./AGV_mail_Router.js";
import config from "./models/oracle/config.js";
import jwt from "jsonwebtoken";
import * as oracledb from "./models/oracle/oraclecon.js";
import WebSocket,{ WebSocketServer } from "ws";
const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);
const oracleRouter = express.Router();
const HEARTBEAT_INTERVAL_TO_client = 20000; // heartbeat 20s
const HEARTBEAT_INTERVAL_Receive_From_webuser = 1000; // heartbeat 20s
let relocation_initpose_bool, path_cancel_bool, go_home_bool, multiple_points_bool, control_mode_bool, chatgpt_msg_bool;
const clientMap = new Map(); // store client id and WebSocket connect
const clientDataMap = new Map(); // store clientID and received data
const heartbeats = new Map(); // erery car ID's heartbeat

// 創建WebSocket伺服器並將其掛載到同一個HTTP服務器上 const server = http.createServer(app);
var app = express();
const wss = new WebSocketServer({ port:3562 });
//websocket && car communication
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
            // 更新 clientDataMap
            const clientData = clientDataMap.get(clientId) || {};
            clientDataMap.set(clientId, {
              ...clientData,
              navigationStatus: receivedData.navigationStatus,
              joystickStatus: receivedData.joystickStatus,
              currentPose: receivedData.currentPose,
              map_match_ratio: receivedData.map_match_ratio,
              startnavigation_status: receivedData.startnavigation_status,
              chatgpt_location_name: receivedData.chatgpt_location_name  // 确保更新这个字段
            });
          } else {
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
//允許cors
const allowedOrigins = [
  "http://localhost:3000",
  "http://192.168.102.38:3000",
  "http://192.168.103.161:3000",
  "http://192.168.103.167:3000",
  "http://192.168.103.94:3000",
  "http://192.168.103.171:3000",
  "http://192.168.103.83:9090",
  "http://192.168.103.174:3001",
];
app.set("superSecret", config.secret);
app.use(
  cors({
    origin: function (origin, callback) {
      // 如果 origin 是 undefined，那麼表示這個請求是來自同源的
      if (!origin) return callback(null, true);

      if (allowedOrigins.indexOf(origin) === -1) {
        var msg =
          "The CORS policy for this site does not " +
          "allow access from the specified Origin.";
        return callback(new Error(msg), false);
      }
      return callback(null, true);
    },
    credentials: true, // 允許發送憑據（如 cookies）
  })
);

// view engine setup
app.set("views", path.join(__dirname, "views"));
app.set("view engine", "jade");

app.use(logger("dev"));
app.use(express.json());
//改成true 才能傳送複合型態資料 如:陣列型態
app.use(express.urlencoded({ extended: true }));
// app.use(cookieParser());
app.use(express.static(path.join(__dirname, "public")));
const apiAllowList = new Set(["/getalluserauth", "/loginJWT"]);
//廠別別對應的員工編號權限
let userFirmMap = new Map([
  ["7", new Set()], //高雄廠
  ["A", new Set()], //漳州廠
]);



 

//刷新廠別與員工編號的Map
async function resetUserFirmMap() {
  let arr = await oracledb.getFirmUser();
  if (arr.length) {
    const tempMap = arr.reduce((map, row) => {
      if (!map.has(row.FIRM)) {
        map.set(row.FIRM, new Set());
      }
      map.get(row.FIRM).add(row.PPS_CODE);
      return map;
    }, new Map());
    // console.log("tempMap: " + mapSetToJSON(tempMap));
    userFirmMap = tempMap;
    // console.log("userFirmMap: " + mapSetToJSON(userFirmMap));
  }
}

oracleRouter.use((req, res, next) => {
  let token = null;
  if (
    req.headers.authorization &&
    "Bearer" === req.headers.authorization.split(" ")[0]
  ) {
    token = req.headers.authorization.split(" ")[1];
  } else {
    token = req.body.token || req.query.token || req.headers["x-access-token"];
  }
  let statusCode = 200;
  let errMsg = "";

  req.user = {};
  if (token) {
    let expireSetting =
      "/loginJWT" === req.path ? {} : { ignoreExpiration: true }; //僅在Refresh時重新驗證Token期限
    jwt.verify(token, app.get("superSecret"), expireSetting, (err, decoded) => {
      if (err || !req.headers.firm) {
        statusCode = 403;
        errMsg = "Failed to authenticate token.";
      } else {
        //如果廠別存在於Map中，則允許繼續
        if (userFirmMap.get(req.headers.firm).has(decoded.PPS_CODE)) {
          if ("7" === req.headers.firm) {
            decoded.COMPANY = "1";
            decoded.FIRM = "7";
            decoded.DEPT = "17P2";
          } else if ("A" === req.headers.firm) {
            decoded.COMPANY = "A";
            decoded.FIRM = "A";
            decoded.DEPT = "AAP1";
          } else {
            statusCode = 403;
            errMsg = "Failed to read firm header.";
          }
        }
        req.user = decoded;
      }
    });
  } else {
    statusCode = 403;
    errMsg = "No token provided.";
  }

  //為了開發方便從localhost進來的連線可以忽略token
  if (String(req.headers.host).startsWith("localhost")) {
    return next();
  } else if ("OPTIONS" === req.method) {
    //方便local測試PDA用
    return next();
  } else if (apiAllowList.has(req.path) > -1) {
    return next();
  } else if (200 !== statusCode) {
    return res.status(statusCode).json({ res: errMsg, error: true });
  }
  return next();
});

oracleRouter.post("/getalluserauth", async (req, res) => {
  const id = req.body.id;
  const pw = req.body.pw;
  const firm = req.body.firm;

  // console.log("firm: " + firm)

  const apivisionUrl = "https://vision.ccpgp.com/api/common/login";
  axios
    .post(apivisionUrl, { id: id, pw: pw }, { proxy: false, timeout: 10000 })
    .then((val) => {
      if (val.data.token && firm) {
        oracledb.getAllUserAuth(val.data.user.PPS_CODE, firm).then((auth) => {
          if (auth !== 0) {
            val.data.authRoutes = auth;
            val.data.firm = firm;
          } else {
            val.data.error = "查無權限";
            val.data.token = null;
          }
          res.send(val.data);
        });
      } else {
        res.send(val.data);
      }
    })
    .catch((err) => {
      //   console.err(libs.getNowDatetimeString(), "Login Error", err.toString());
      res.send(err);
    });
});


//聲明物件 要放置在上面中間件後面
app.use("/newmember", memberRouter);
app.use("/public", express.static("public")); //運用static 將public 目錄對外開放 讓城市透過參數存取目錄中的靜態檔案
app.use('/AGVMail', AgvMailRouter);
app.use("/oracle", oracleRouter);


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
   const clientData = clientDataMap.get(targetClientId);
   chatgpt_msg_bool = true;
 
   if (targetClient && targetClient.readyState === WebSocket.OPEN) {
     targetClient.send(
       JSON.stringify({
         chatgpt_msg_bool: chatgpt_msg_bool,
         chatgpt_msg: chatgpt_msg,
       })
     );
     console.log("Received chatgpt_msg value:", chatgpt_msg);
 
     // 延迟 2 秒后发送响应
     setTimeout(() => {
       // 重新获取 clientData 以确保是最新的
       const updatedClientData = clientDataMap.get(targetClientId);
       res.status(200).send({ chatgpt_location_name: updatedClientData?.chatgpt_location_name });
     }, 2000);  // 延迟 2 秒
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
 

///設定Server的Port
app.listen(3561, () => {
  resetUserFirmMap();
  console.log("server started");
});
