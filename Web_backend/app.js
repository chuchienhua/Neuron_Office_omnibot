import memberRouter from "./routes/Mongo_memberApi.js";
import axios from "axios";
import express from "express";
import path from "path";
// import cookieParser from "cookie-parser";
import logger from "morgan";
import cors from "cors";
import { fileURLToPath } from "url";
import { dirname } from "path";
import config from "././models/oracle/config.js";
import jwt from "jsonwebtoken";
import * as oracledb from "./models/oracle/oraclecon.js";
const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);
const oracleRouter = express.Router();
var app = express();
const allowedOrigins = [
  "http://localhost:3000",
  "http://192.168.102.38:3000",
  "http://192.168.103.161:3000",
  "http://192.168.103.167:3000",
  "http://192.168.103.94:3000",
  "http://192.168.103.171:3000",
  "http://192.168.103.83:9090",
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

//console出map資訊
function mapSetToJSON(map) {
  const obj = {};
  for (let [key, value] of map.entries()) {
    obj[key] = Array.from(value);
  }
  return JSON.stringify(obj);
}

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

app.use("/oracle", oracleRouter);


///設定Server的Port
app.listen(3003, () => {
  resetUserFirmMap();
  console.log("server started");
});
