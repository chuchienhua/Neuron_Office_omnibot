import React, { useState, useEffect } from "react";
import { Link, useNavigate } from "react-router-dom";
import { useSelector, useDispatch } from "react-redux";
import { registerAllCellTypes } from "handsontable/cellTypes";
import axios from "axios";
import Utils from "../../Utils.js"
import "./HomeOracle.css";
import "./Car_Interface_2.css";
import initialPoints from "../Map/InitialPoints.js";
import quaternionToYaw from "../TFconvert/quaternionToYaw.js";
import { saveCarList, pathCar_cancel } from "../Goalhistory/GoalList.js";
import { toast } from "react-toastify";
function Car_Interface_2({ onMapData }) {
  const user = useSelector((state) => state.user);
  registerAllCellTypes();
  const navigate = useNavigate();
  const dispatch = useDispatch();
  const [AppTabs, setAppTabs] = useState("CarInfo");
  const [go_home_msg, set_go_home_msg] = useState([""]);
  const [relocation_initpose_msg, set_relocation_initpose_msg] = useState([""]);
  const [number, setNumber] = useState({
    position: { x: null, y: null },
    orientation: { z: null, w: null },
  });

  const [FirstConnect_check, setFirstConnect_check] = useState(false);
  const [carYaw, setCarYaw] = useState(null);
  const [isPolling, setIsPolling] = useState(false);
  const [carConnect, setCarConnect] = useState(false);
  const [carConnectMsg, setCarConnectMsg] = useState("");
  const [start_navigation, setStart_navigation] = useState(false);
  const [navigatgionStatusMsg, setNavigatgionStatusMsg] = useState(
    "顯示自動導航目前狀態"
  );
  const [Site, setSite] = useState(0);
  const [connect_count, setConnect_count] = useState(0);
  const [Dispatch_x, setDispatch_x] = useState("");
  const [Dispatch_y, setDispatch_y] = useState("");
  const [Dispatch_z, setDispatch_z] = useState("");
  const [Dispatch_Name, setDispatch_Name] = useState("");
  const [Savecarpoint, setSavecarpoint] = useState([]);
  const [selectedIndex, setSelectedIndex] = useState(null);
  const [records_carpoint, setRecords_carpoint] = useState([]);
  const [Fin_records_carpoint, setFin_Records_carpoint] = useState([]);
  const [isExpanded, setIsExpanded] = useState(null);
  const [car_mode, setcar_mode] = useState(true);
  const [mapX, setMapX] = useState("");
  const [mapY, setMapY] = useState("");
  const [mapyaw, setMapyaw] = useState("");
  const [controlmode, setControlmode] = useState("");
  const [searchMode, setSearchMode] = useState("Dispatch_Mode");
  const [searchTerm, setSearchTerm] = useState("");
  const [map_match_ratio, setMap_match_ratio] = useState("");
  const [Is_map_match, setIs_map_match] = useState(true);
  const [lastMapMatchRatio, setLastMapMatchRatio] = useState(null);
  const [sameRatioCount, setSameRatioCount] = useState(0);
  const [connect_heartbeat, setConnect_heartbeat] = useState(0);
  const map_match_parameter = 0.5;
  // const Carurl = "http://192.168.103.171:3000/";
  const Carurl = "http://192.168.8.241:3561/"; //TP_IOT

  const trStyle = (index, selectedIndex) => ({
    backgroundColor: selectedIndex === index ? "#f2f2f2" : "#fff",
    cursor: "pointer", // Add pointer on hover over the rows
  });

  //增加點位
  const addPoint = () => {
    const newPoint = {
      name: String(Dispatch_Name),
      site: Number(Site),
      x: Number(Dispatch_x),
      y: Number(Dispatch_y),
      yaw: Number(Dispatch_z),
    };
    setSite((prevSite) => prevSite + 1);
    setSavecarpoint([...Savecarpoint, newPoint]);
    setDispatch_Name("");
    setDispatch_x("");
    setDispatch_y("");
    setDispatch_z("");
  };

  //刪除選中的點
  const deleteSelectedPoint = () => {
    if (selectedIndex !== null) {
      // 1. 過濾選中的點
      const newSavecarpoint = Savecarpoint.filter(
        (_, index) => index !== selectedIndex
      );

      // 2. 更新剩餘site
      const updatedSavecarpoint = newSavecarpoint.map((point, index) => ({
        ...point,
        site: index, // 重新編號site
      }));

      // 3. 更新state
      setSite(updatedSavecarpoint.length); // 更新site數目
      setSavecarpoint(updatedSavecarpoint); // 更新點列表

      // 4. 如果刪除最後一個點，selectedIndex設置為null或最後一个元素的索引
      if (selectedIndex >= updatedSavecarpoint.length) {
        setSelectedIndex(updatedSavecarpoint.length - 1);
      } else {
        // 保持當前的selectedIndex不變，除非現在超出了新數组的長度
        setSelectedIndex((prev) =>
          prev >= updatedSavecarpoint.length ? null : prev
        );
      }
    }
  };

  //清除點位
  const clearPointList = () => {
    setSavecarpoint([]);
    setSite(0);
    setSelectedIndex(null);
  };
  // 修改按鈕的事件處理器
  const handleConnectButtonClick = () => {
    setCarConnect(!carConnect);
  };

  useEffect(() => {
    if (onMapData && typeof onMapData === 'function') {
      onMapData(mapX, mapY, mapyaw);
    }
  }, [mapX, mapY, mapyaw, onMapData]);

  //建立連線
  useEffect(() => {
    let connectIntervalId;
    let lastDataReceivedTime = Date.now(); // 初始化最後一次收到數據的時間
    let response;
    const connect = async () => {
      // 更新心跳计数器并立即使用新值
      setConnect_heartbeat(prev => {
        const updatedHeartbeat = prev >= 50 ? 0 : prev + 1;
        sendHeartbeat(updatedHeartbeat); // 使用新值发送心跳
        return updatedHeartbeat;
      });
    };

    // 封装心跳发送逻辑
    const sendHeartbeat = async (heartbeatValue) => {
      try {
        const response = await axios.post(Carurl + "connect", {
          carid: 0x02,
          carconnect_bool: true,
          connect_heartbeat: heartbeatValue
        });
        lastDataReceivedTime = Date.now(); // 成功接收數據時更新時間
        setCarConnectMsg(response.data.connect); // 更新状态
        setStart_navigation(response.data.startnavigation_status)
        // setNavigatgionStatusMsg(response.data.navigationStatus); // 更新状态
        // setControlmode(response.data.joystickStatus);
        // console.log(response);
        if (JSON.stringify(response.status) === "200") {
          setFirstConnect_check(true);
        }
      } catch (error) {
        console.error("Could not fetch data", error);
        setConnect_count(prev => prev + 1);
      }
    };
    const disconnect = async () => {
      try {
        response = await axios.post(Carurl + "disconnect", { carid: 0x02, carconnect_bool: false });
        console.log(response.data.disconnect);
        window.location.reload();
      } catch (error) {
        console.error("Could not fetch data", error);
      }
    };

    if (carConnect) {
      setIsPolling(true);
      connect(); // 立即調用一次
      connectIntervalId = setInterval(connect, 800); // 每 4000 毫秒調用一次
      // 檢查是否超過4.5秒未收到數據
      const checkDataTimeout = setInterval(() => {
        if (Date.now() - lastDataReceivedTime > 2000 && !FirstConnect_check) {
          // 4秒超時
          setCarConnect(false); // 超時，將carConnect設為false
          setIsPolling(false);
          // console.log("超時");
          // 清除定時器
          return () => {
            clearInterval(connectIntervalId);
            clearInterval(checkDataTimeout); // 清除檢查超時定時器
          };
        }
      }, 1000); // 每秒檢查一次
      // 清除定時器
      return () => {
        clearInterval(connectIntervalId);
        clearInterval(checkDataTimeout); // 清除檢查超時定時器
      };
    } else {
      disconnect();
      setFirstConnect_check(false);
      setIsPolling(false);
      setNavigatgionStatusMsg("顯示自動導航目前狀態");
      setDispatch_Name("");
      setDispatch_x("");
      setDispatch_y("");
      setDispatch_z("");
      setNumber({
        position: {
          x: null,
          y: null,
        },
        orientation: {
          z: null,
          w: null,
        }
      })
      clearPointList();
      setConnect_count(0); //初始 connect_count
      setSameRatioCount(0); //防止斷掉後 值還在繼續送
      setMap_match_ratio(''); //初始 map_match_ratio
      setConnect_heartbeat(0);
    }
  }, [carConnect]);

  //收集車子位置資訊 / 地圖匹配率
  useEffect(() => {
    let intervalId;
    if (isPolling) {
      const fetchNumber = async () => {
        try {
          const response = await axios
            .post(Carurl + "current_pose", {
              carid: 0x02,
            });
          const data = response.data;
          setNavigatgionStatusMsg(data.navigationStatus); // 更新状态
          setControlmode(data.joystickStatus);
          const newMapMatchRatio = data.map_match_ratio;
          // 防止car server端已斷線後，前端還在收取map_match_ratio的連續相同值
          if (newMapMatchRatio === lastMapMatchRatio) {
            setSameRatioCount(prev => prev + 1);
            if (sameRatioCount >= 1000) {
              setCarConnect(false);
            }
          } else {
            setSameRatioCount(0);
          }
          setLastMapMatchRatio(newMapMatchRatio);
          setMap_match_ratio(newMapMatchRatio);
          // console.log("map_ratio: " + newMapMatchRatio);
          setNumber({
            position: {
              x: parseFloat(data.currentPose.position.x.toFixed(6)),
              y: parseFloat(data.currentPose.position.y.toFixed(6)),
            },
            orientation: {
              z: data.currentPose.orientation.z,
              w: data.currentPose.orientation.w,
            },
          }); // 更新車子位置狀態

        } catch (error) {
          console.error("Could not fetch number", error);
          setConnect_count((prev) => prev + 1);//確認在錯誤時是否正常斷線
          if (connect_count > 300) {
            setCarConnect(false);
          }
        }
      };
      fetchNumber(); // 使用並設置計時器
      intervalId = setInterval(fetchNumber, 800); // 每 100 毫秒调用一次
    }
    // 清除定时器
    return () => {
      if (intervalId) {
        clearInterval(intervalId);
      }
    };
  }, [isPolling, connect_count, lastMapMatchRatio, sameRatioCount]);

  //確認地圖是否匹配
  useEffect(() => {
    if (map_match_ratio <= map_match_parameter) {
      setIs_map_match(false);
    } else {
      setIs_map_match(true);
    }
  }, [map_match_ratio])

  //確認導航中是否斷線
  useEffect(() => {
    console.log(start_navigation);
    if (start_navigation) {
      if (!carConnect) {
        console.log("導航中斷線");
        AGV_problem_mail();
      }
    }

  }, [carConnect, start_navigation])

  //位置 TF 4元素轉換
  useEffect(() => {
    let newCarYaw = quaternionToYaw(number.orientation.z, number.orientation.w);
    let mappose = convertCoordinates(
      number.position.x,
      number.position.y,
      newCarYaw
    );
    // let mappose = convertCoordinates(-3.604,-7.748,0);
    setCarYaw(parseFloat(newCarYaw).toFixed(6));
    setMapX(mappose.mapX);
    setMapY(mappose.mapY);
    setMapyaw(mappose.mapyaw);
  }, [number]);

  //回家訊息
  useEffect(() => {
    const timer = setTimeout(() => {
      set_go_home_msg("");
    }, 10000);
    return () => clearTimeout(timer);
  }, [go_home_msg]);

  //重定位訊息
  useEffect(() => {
    const timer = setTimeout(() => {
      set_relocation_initpose_msg("");
    }, 10000);
    return () => clearTimeout(timer);
  }, [relocation_initpose_msg]);

  //回家(0,0,0)
  const Car_Go_Home = () => {
    axios
      .post(Carurl + "go_home", {
        carid: 0x02,
        go_home_value: 40, //ROS value setting Int8 40
      })
      .then((res) => {
        set_go_home_msg(res.data);
      })
      .catch((err) => {
        console.log(err);
      });
  };

  //控制模式切換
  const Car_control_mode = () => {
    setcar_mode(!car_mode);
    axios
      .post(Carurl + "control_mode", {
        carid: 0x02,
        control_mode: 42, //ROS value setting Int8 40
      })
      .then((res) => {
        set_go_home_msg(res.data);
      })
      .catch((err) => {
        console.log(err);
      });
  };

  //重定位地圖(0,0,0)
  const Relocation_initpose = () => {
    axios
      .post(Carurl + "relocation_initpose", {
        carid: 0x02,
        relocation_initpose_value: 38, //ROS value setting 38
      })
      .then((res) => {
        set_relocation_initpose_msg(res.data);
      })
      .catch((err) => {
        console.log(err);
      });
  };

  //格式化時間
  const formatDate = (dateString) => {
    const date = new Date(dateString);
    return date.toLocaleString('zh-TW', {
      hour12: false, // 使用24小時制
      year: 'numeric',
      month: 'numeric',
      day: 'numeric',
      hour: '2-digit',
      minute: '2-digit',
      second: '2-digit',
    });
  };

  //展開歷史紀錄
  const toggleExpand = (index) => {
    setIsExpanded(isExpanded === index ? null : index);
  };
  //------------------

  //車子派遣
  const Car_Set_Goal = () => {
    if (Is_map_match) {
      axios
        .post(Carurl + "set_goal", {
          carid: 0x02,
          multiple_points: Savecarpoint, //多點位
        })
        .then((res) => {
          console.log(res.status);
        })
        .catch((err) => {
          console.log(err);
        });
      saveCarList(Savecarpoint);
    } else {
      toast.error("地圖匹配率過低，請重新校正車子位置");
    }
  };

  //取消路徑
  const Car_Path_Cancel = () => {
    axios
      .post(Carurl + "path_cancel", {
        carid: 0x02,
        path_cancel_value: 34, //ROS value setting 34
      })
      .then((res) => {
        console.log(res.status);
      })
      .catch((err) => {
        console.log(err);
      });
    pathCar_cancel();
  };

  //郵件測試
  const AGV_problem_mail = () => {
    axios
      .post(Carurl + "AGVMail/sendmail", {
        carid: 0x01,
        user: user.PPS_CODE, //ROS value setting 34
      })
      .then((res) => {
        console.log(res.status);
      })
      .catch((err) => {
        console.log(err);
      });
  };

  const moveRow = (currentIndex, direction) => {
    setSavecarpoint((prevState) => {
      let newArray = [...prevState]; // 创建数组的副本

      // 确定目標索引
      let targetIndex =
        direction === "up" ? currentIndex - 1 : currentIndex + 1;

      // 檢查目標索引是否有效
      if (targetIndex >= 0 && targetIndex < newArray.length) {
        // 交换x、y和yaw值
        [newArray[currentIndex].name, newArray[targetIndex].name] = [
          newArray[targetIndex].name,
          newArray[currentIndex].name,
        ];
        [newArray[currentIndex].x, newArray[targetIndex].x] = [
          newArray[targetIndex].x,
          newArray[currentIndex].x,
        ];
        [newArray[currentIndex].y, newArray[targetIndex].y] = [
          newArray[targetIndex].y,
          newArray[currentIndex].y,
        ];
        [newArray[currentIndex].yaw, newArray[targetIndex].yaw] = [
          newArray[targetIndex].yaw,
          newArray[currentIndex].yaw,
        ];
        setSelectedIndex(targetIndex);
      }
      return newArray; // 返回新的数组副本作为新状态
    });
  };

  // 模擬地圖座標轉換
  const convertCoordinates = (x, y, yaw) => {
    return {
      mapX: 37 + y * -6.0, //37為我的(0,0,0)地圖座標
      mapY: 24 + x * -6.8, //24為我的(0,0,0)地圖座標
      mapyaw: yaw * (-180 / Math.PI),
    };
  };

  // 搜尋與過濾
  useEffect(() => {
    let results = records_carpoint;
    if (searchTerm) {
      if (searchMode === "Dispatch_Mode") {
        results = records_carpoint.filter(record =>
          record.status.toLowerCase().includes(searchTerm.toLowerCase())
        );
      } else if (searchMode === "Time") {
        results = records_carpoint.filter((record) => {
          const formattedRecordTime = formatDate(record.time);
          return formattedRecordTime.includes(searchTerm);
        });
      }
    }
    setFin_Records_carpoint(results);
  }, [searchMode, searchTerm, records_carpoint]);


  const switchbutton = (switchcase) => {
    if (switchcase === "CarInfo") {
    } else if (switchcase === "GoalHistory") {
    }
  };

  return (
    <div className="custom-background">
      <div className="copy_content-wrapper">
        <div className=" tabs-container">
          <div className="ms-1">
            <button
              type="button"
              className={
                carConnect
                  ? "btn btn-success btn-lg"
                  : "btn btn-outline-danger btn-lg"
              }
              onClick={() => handleConnectButtonClick()} //setCarConnect(!carConnect)
            >
              {carConnect ? "已連線" : "連線"}
            </button>
            <span
              className={
                carConnect
                  ? FirstConnect_check
                    ? "connect-success-style-font ms-2"
                    : "no-connect-success-style-font ms-2"
                  : "no-connect-success-style-font ms-2"
              }
            >
              {carConnect
                ? FirstConnect_check
                  ? carConnectMsg.toString()
                  : "請確認Server正在運行，4秒後將斷線" //Make sure the car server is running, 4 Seconds Will be Disconnected
                : "請按連線，連線成功後才能進行以下操作" //Please Click Connect to Car, Make sure to do the following.
              }
            </span>
          </div>
          <div>
            <button
              type="button"
              className="btn btn-outline-danger mt-1 ms-1 "
              //連上線才能開始偵測
              onClick={() => setIsPolling(!isPolling)}
            >
              {isPolling && carConnect
                ? "停止偵測車體資訊" //Stop detecting Infomation
                : "開始偵測車體資訊" //Start detecting Infomation
              }
            </button>
            <span className={"Map-match-style-font ms-2 mt-3"}>
              {`地圖匹配率 : ${map_match_ratio}`}
            </span>
            <div className="position-container mt-1 ms-1">
              <input
                type="text"
                className="control-position-input"
                value={`控制模式 : ${carConnect ? (controlmode ? " 手動控制 " : " 自動控制 ") : " 等待連線 "}`}
              />
              <input
                type="text"
                className="position-input ms-2 "
                value={`X方向 :  ${number.position.x}`}
                readOnly
              />
              <input
                type="text"
                className="position-input ms-2"
                value={`Y方向 :  ${number.position.y}`}
                readOnly
              />
              <input
                type="text"
                className="position-input ms-2"
                value={`Yaw方向 :  ${Number(carYaw).toFixed(6)}`}
                readOnly
              />
            </div>
          </div>
          <div className="position-container mt-1 ms-1">
            <div className="input-group">
              <span className="navigation-status-style-font mt-1">
                自動導航狀態:
              </span>
              <input
                type="text"
                aria-label="X"
                className="navigation-form-control ms-2"
                placeholder="目前導航狀態 "
                value={navigatgionStatusMsg}
              />
            </div>
          </div>
          <div className="mt-2 ms-1 button-input-container">
            <button
              type="button"
              className="btn btn-success my-button "
              onClick={() => {
                if (FirstConnect_check) {
                  Car_Go_Home();
                }
              }}
            >
              回到原點
            </button>
            {/* <input
                  type="text"
                  className="form-control ms-1"
                  placeholder="回到原點 (X, Y, Yaw) = (0,0,0)" //"Msg :"
                  value={go_home_msg}
                  readOnly
                /> */}
            <button
              type="button"
              className="btn btn-info my-button  ms-3"
              onClick={() => {
                if (FirstConnect_check) {
                  Relocation_initpose();
                }
              }}
            >
              重新定位
            </button>
            <button
              type="button"
              className="btn btn-warning my-button  ms-3"
              onClick={() => {
                if (FirstConnect_check) {
                  Car_control_mode();
                }
              }}
            >
              控制模式切換
            </button>
            {/* <input
                  type="text"
                  className="form-control small-input ms-2"
                  placeholder="重新定位地圖在 (X, Y, Yaw) = (0,0,0)"
                  value={relocation_initpose_msg}
                  readOnly
                /> */}
          </div>
          <div>
            <button
              type="button"
              className="btn btn-outline-info btn-lg mt-2 ms-2"
              onClick={() => {
                // if (FirstConnect_check) {
                addPoint();
                // }
              }}
            >
              新增停靠位置
            </button>
            {/* <span className="navigathon-statuss-style-font ms-2">
                  {navigatgionStatusMsg}
                </span> */}
          </div>
          <div className="position-container mt-1 ms-2">
            <div className="input-group">
              <input
                type="text"
                aria-label="X"
                className="form-control"
                placeholder="座位人員姓名"
                value={Dispatch_Name}
                readOnly
              />
              <input
                type="text"
                aria-label="X"
                className="form-control"
                placeholder="目標座標 X "
                value={Dispatch_x}
                onChange={(e) => setDispatch_x(e.target.value)}
              />
              <input
                type="text"
                aria-label="Y"
                className="form-control"
                placeholder="目標座標 Y "
                value={Dispatch_y}
                onChange={(e) => setDispatch_y(e.target.value)}
              />
              <input
                type="text"
                aria-label="Z"
                className="form-control"
                placeholder="目標座標 Yaw "
                value={Dispatch_z}
                onChange={(e) => setDispatch_z(e.target.value)}
              />
            </div>
          </div>

          <div className="tableContainerStyle mt-3">
            <h3>
              派車點位停靠列表:
              <button
                className="btn btn-outline-danger  ms-4"
                onClick={() => {
                  if (FirstConnect_check) {
                    deleteSelectedPoint();
                  }
                }}
              >
                刪除點位
              </button>
              <button
                className="btn btn-outline-danger  ms-2"
                onClick={() => {
                  if (FirstConnect_check) {
                    clearPointList();
                  }
                }}
              >
                清空列表點位
              </button>
              {/* {selectedIndex != null && ( */}
              <>
                <button
                  className="btn btn-outline-secondary ms-2"
                  onClick={() => moveRow(selectedIndex, "up")}
                  disabled={selectedIndex === 0} // 如果已經是第一個元素，禁用上移按鈕
                >
                  上移
                </button>
                <button
                  className="btn btn-outline-secondary ms-2"
                  onClick={() => moveRow(selectedIndex, "down")}
                  disabled={selectedIndex === Savecarpoint.length - 1} // 如果已經是最後一個元素，禁用下移按鈕
                >
                  下移
                </button>
              </>
              {/* )} */}
            </h3>
            <div className="tableStyle">
              <table>
                <thead>
                  <tr>
                    <th className="thTdStyle">Name :</th>
                    <th className="thTdStyle">Site :</th>
                    <th className="thTdStyle">X :</th>
                    <th className="thTdStyle">Y :</th>
                    <th className="thTdStyle">Yaw :</th>
                  </tr>
                </thead>
                <tbody>
                  {Savecarpoint.map((point, index) => (
                    <tr
                      key={index}
                      onClick={() => setSelectedIndex(index)}
                      style={trStyle(index, selectedIndex)}
                    >
                      <td className="thTdStyle">{point.name}</td>
                      <td className="thTdStyle">{point.site}</td>
                      <td className="thTdStyle">{point.x}</td>
                      <td className="thTdStyle">{point.y}</td>
                      <td className="thTdStyle">{point.yaw}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          </div>
          <button
            type="button"
            className="btn btn-outline-success btn-lg mt-3 ms-2"
            onClick={() => {
              //確保連線上 且 車子 math 地圖
              if (FirstConnect_check) {
                Car_Set_Goal();
              }
            }}
          >
            任務出發
          </button>
          <button
            type="button"
            className="btn btn-outline-danger btn-lg mt-3 ms-2"
            onClick={() => {
              if (FirstConnect_check) {
                Car_Path_Cancel();
              }
            }}
          >
            任務取消
          </button>
        </div>
      </div>
    </div>
  );
}

export default Car_Interface_2;
