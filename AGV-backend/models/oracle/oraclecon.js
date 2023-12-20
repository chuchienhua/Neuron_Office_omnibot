import config from "./config.js";
import oracledb from "oracledb";
import { getNowDatetimeString } from "../../lib.js";


export async function getAllUserAuth(userppscode, firm) {
  let connectionOracle;
  let rows = [];
  // console.log("firm111: " + firm)

  try {
    connectionOracle = await oracledb.getConnection(config.ORACLE_CONFIG);

    const sql = `SELECT S0.ROUTE,S0.ISADMIN,S1.ROUTE_NAME,S1.IS_HIDE
                 FROM PBTC_IOT_AUTH S0 
                 LEFT JOIN PBTC_IOT_ROUTE_SETTINGS S1
                 ON S0.ROUTE = S1.ROUTE
                 WHERE S0.PPS_CODE = :PPS_CODE 
                 AND S0.FIRM = :FIRM
                 ORDER BY S1.ROUTE_ORDER`;
    const params = {
      PPS_CODE: {
        dir: oracledb.BIND_IN,
        type: oracledb.STRING,
        val: "" + userppscode,
      },
      FIRM: {
        dir: oracledb.BIND_IN,
        type: oracledb.STRING,
        val: "" + firm,
      },
    };
    const options = { outFormat: oracledb.OUT_FORMAT_OBJECT };
    const result = await connectionOracle.execute(sql, params, options);
    rows = result.rows;
  } catch (err) {
    console.log(err);
  } finally {
    if (connectionOracle) {
      await connectionOracle.close();
    }
  }
  return rows;
}

//取得所有廠擁有功能的員工編號
export async function getFirmUser() {
  let conn;
  let rows = [];
  try {
      conn = await oracledb.getConnection(config.ORACLE_CONFIG);

      const sql = `
          SELECT FIRM, PPS_CODE
          FROM PBTC_IOT_AUTH
          GROUP BY FIRM, PPS_CODE `;
      const result = await conn.execute(sql, {}, { outFormat: oracledb.OBJECT });
      rows = result.rows;
  } catch (err) {
      console.error(getNowDatetimeString(), 'getFirmUser', err);
  } finally {
      if (conn) {
          await conn.close();
      }
  }
  return rows;
}
