import React, { useState, useEffect } from "react";
import "./HomeOracle.css";
import axios from "axios";
import Tab from "react-bootstrap/Tab";
import Tabs from "react-bootstrap/Tabs";
import { useNavigate } from "react-router-dom";
import { useSelector, useDispatch } from "react-redux";
import Utils from "../../Utils";
import { registerAllCellTypes } from "handsontable/cellTypes";
import { HotTable } from "@handsontable/react";
import Handsontable from "handsontable";
import quaternionToYaw from "../TFconvert/quaternionToYaw";
function HomeOracle() {
  registerAllCellTypes();
  const navigate = useNavigate();
  const dispatch = useDispatch();
  const user = useSelector((state) => state.user);
  const firm = useSelector((state) => state.firm);
  const [AppTabs, setAppTabs] = useState("Home");
  const [material, setMaterial] = useState("*");
  const [getAuthMaterialdata, setgetAuthMaterialdata] = useState([]);
  const [remainderRows, setRemainderRows] = useState([]);
  const [go_home_msg, set_go_home_msg] = useState([""]);
  const [relocation_initpose_msg, set_relocation_initpose_msg] = useState([""]);
  const [number, setNumber] = useState({
    position: { x: null, y: null },
    orientation: { z: null, w: null },
  });
 

  // console.log(getAuthMaterialdata);
  const columns = [
    { data: "MATERIAL", type: "text", width: 150, readOnly: true },
    { data: "WEIGHT", type: "numeric", width: 200 },
    { data: "BATCH_NO", type: "text", width: 200 },
    { data: "LOT_NO", type: "text", width: 150 },
    {
      data: "SAVE_BTN",
      width: 65,
      readOnly: true,
      renderer: function (instance, td, row, col, prop, value, cellProperties) {
        const icon = document.createElement("SPAN");
        icon.className = "icon bi-save";
        icon.innerHTML = " 儲存";

        const btn = document.createElement("SAVE_BTN");
        btn.className =
          "btn btn-outline-success btn-sm px-1 py-0 nowrap align-top";
        btn.appendChild(icon);

        Handsontable.dom.addEvent(btn, "click", () => {
          // saveBtn(row);
        });

        const div = document.createElement("DIV");
        div.appendChild(btn);

        Handsontable.dom.empty(td);
        td.appendChild(div);
      },
    },
    {
      data: "LABEL_BTN",
      width: 65,
      readOnly: true,
      renderer: function (instance, td, row, col, prop, value, cellProperties) {
        const icon = document.createElement("SPAN");
        icon.className = "icon bi-printer";
        icon.innerHTML = " 列印";

        const btn = document.createElement("LABEL_BTN");
        btn.className =
          "btn btn-outline-secondary btn-sm px-1 py-0 nowrap align-top";
        btn.appendChild(icon);

        Handsontable.dom.addEvent(btn, "click", () => {
          // labelBtn(row);
        });

        const div = document.createElement("DIV");
        div.appendChild(btn);

        Handsontable.dom.empty(td);
        td.appendChild(div);
      },
    },
  ];

  const colHeader = ["原料名稱", "餘料量", "棧板編號", "批號", "更新", "標籤"];

  const switchbutton = (switchcase) => {
    if (switchcase === "Home") {
      getAuthMaterial();
    } else if (switchcase === "Profile") {
      getMaterial();
    } else if (switchcase === "CarInfo") {
    }
  };

  const getAuthMaterial = () => {
    const apiurl = Utils.getURL("oracle/getAutuMaterial");
    axios
      .post(apiurl, {
        ppd_code: user.PPS_CODE,
      })
      .then((res) => {
        // console.log(res.data);
        setgetAuthMaterialdata(res.data);
      })
      .catch((err) => {
        console.log(err);
      });
  };

  const getMaterial = () => {
    const apiurl = Utils.getURL("oracle/getMaterial", [material]);
    axios
      .get(apiurl, {
        ...Utils.pbtAxiosConfig,
      })
      .then((res) => {
        setRemainderRows(res.data.res);
      })
      .catch((err) => {
        console.log(err);
      });
  };

  return (
    <div className="custom-background">
      {AppTabs === "CarInfo" ? null : (
        <div className="ms-2">
          <button
            type="button"
            className="btn btn-outline-success mt-1  "
            onClick={() => {
              switchbutton(AppTabs);
            }}
          >
            <span className="icon bi-plus-circle"></span> 查詢
          </button>
          <button type="button" className="btn btn-outline-info mt-1 ms-2 ">
            <span className="icon bi-plus-circle"></span> 新增
          </button>
          <button type="button" className="btn btn-outline-danger mt-1 ms-2 ">
            <span className="icon bi-plus-circle"></span> 刪除
          </button>
        </div>
      )}
      <div className="mt-1">
        <Tabs
          defaultActiveKey="Home"
          onSelect={(key) => {
            setAppTabs(key);
          }}
          id="uncontrolled-tab-example"
          className="mb-3 custom-tab-border"
        >
          <Tab eventKey="Home" title="Home">
            {AppTabs === "Home" ? (
              <table
                className="table table-striped  table-bordered"
                style={{
                  width: "auto",
                  textAlign: "center",
                  border: "2px solid #000",
                }}
              >
                <thead>
                  <tr>
                    <th>PPS_CODE</th>
                    <th>EDIT_DATE</th>
                    <th>CODE</th>
                    <th>QA</th>
                    <th>QC</th>
                  </tr>
                </thead>
                <tbody>
                  {Array.isArray(getAuthMaterialdata) &&
                  getAuthMaterialdata.length > 0 ? (
                    getAuthMaterialdata.map((item, index) => (
                      <tr key={index}>
                        <td style={{ width: "150px" }}>{item.PPS_CODE}</td>
                        <td style={{ width: "250px" }}>{item.EDIT_DATE}</td>
                        <td style={{ width: "150px" }}>{item.CODE}</td>
                        <td style={{ width: "100px" }}>{item.QA}</td>
                        <td style={{ width: "100px" }}>{item.QC}</td>
                      </tr>
                    ))
                  ) : (
                    <tr>
                      <td colSpan="5" style={{ textAlign: "center" }}>
                        查無資料
                      </td>
                    </tr>
                  )}
                </tbody>
              </table>
            ) : null}
          </Tab>
          <Tab eventKey="Profile" title="Profile">
            {AppTabs === "Profile" ? (
              <HotTable
                filter={true}
                dropdownMenu={true}
                className="handsontable"
                style={{ textAlign: "center" }}
                licenseKey="non-commercial-and-evaluation"
                data={remainderRows}
                columns={columns}
                colHeaders={colHeader}
                rowHeaders={false}
                // hiddenColumns={
                //   {
                //     columns: "material" === tabName ? [1, 2, 3, 4] : [],
                //   }
                // }
              />
            ) : null}
          </Tab>
        </Tabs>
      </div>
    </div>
  );
}

export default HomeOracle;
