import React, { useState, useEffect } from "react";
import "./HomeOracle.css";
import axios from "axios";
import { useNavigate } from "react-router-dom";
import { useSelector, useDispatch } from "react-redux";
import { registerAllCellTypes } from "handsontable/cellTypes";
function LoginHome() {
  registerAllCellTypes();
  const navigate = useNavigate();
  const dispatch = useDispatch();

  return (
    <div className="custom-background">
      <div className="ms-2">
        {/* <button
          type="button"
          className="btn btn-outline-success mt-1  "
          onClick={() => navigate("/homeoracle")}
        >
          <span className="icon bi-plus-circle"></span> Home_Oracle
        </button> */}
        <button
          type="button"
          className="btn btn-outline-danger mt-2 ms-2 "
          onClick={() => navigate("/car_interface")}
        >
          <span className="icon bi-plus-circle"></span> Car_Interface
        </button>
      </div>
    </div>
  );
}

export default LoginHome;
