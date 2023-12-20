import React, { useState } from "react";
import axios from "axios";
import { useNavigate } from "react-router-dom"; // Replace with the appropriate import for your router library
import { useDispatch } from "react-redux"
import Utils from "../../Utils";
import "./LoginOracle.css";
import { toast } from "react-toastify";


function LoginOracle() {
  const dispatch = useDispatch();
  const [id, setUsername] = useState("");
  const [pw, setPassword] = useState("");
  // const [token, setToken] = useState("");
  const [firm, setFirm] = useState("7");
  const navigate = useNavigate();
  const loginHandle = () => {
    axios.defaults.withCredentials = true;
    if (id === "" || pw === "") {
      alert("請輸入帳號密碼");
      return;
    }
    const apiurl = Utils.getURL("oracle/getalluserauth");
    axios
      .post(apiurl, {
        id: id,
        pw: pw,
        firm: firm,
      })
      .then((response) => {
        if(response.data.token){
          console.log(response.data);
          dispatch({type: "LOGIN_SUCCESS", payload: response.data})
          // alert("登入成功!");
          toast.success("登入成功!");
          navigate("/LoginHome")
        }else{
          // alert("登入失敗!");
          toast.error("登入失敗!");
          dispatch({ type: "LOGIN_FAILURE" });
        }
      }).catch((err) => {
        console.log(err);
      });
  };

  return (
    <div className="contentArea">
      <div className="login-block">
        <h1 style={{ color: "blue", textAlign: "center", fontSize: 46 }}>
          Car Dispatch System
        </h1>
        <label style={{ fontSize: 26 }}>Account</label>
        <input
          placeholder="UserName"
          className="form-control"
          type="text"
          onChange={(e) => setUsername(e.target.value)}
          value={id}
        />
        <br />
        <label style={{ fontSize: 26 }}>Password</label>
        <input
          placeholder="Password"
          className="form-control"
          type="password"
          onChange={(e) => setPassword(e.target.value)}
          value={pw}
        />
        <br />
        <button className="btn-signin" onClick={loginHandle}>
          登入
        </button>
      </div>
    </div>
  );
}

export default LoginOracle;
