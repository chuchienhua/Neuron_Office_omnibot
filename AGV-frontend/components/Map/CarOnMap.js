// CarOnMap.js
import React, { memo } from "react";
import carimage from "./car.png";
import "./CarOnMap.css";

const CarOnMap = memo(({ mapX, mapY, yaw }) => {
  return (
    <div
      className="car-on-map"
      style={{
        top: mapY, // 使用正确的单位
        left: mapX, // 使用正确的单位
        backgroundImage: `url(${carimage})`,
        transform: `translate(-50%, -50%) rotate(${yaw}deg)`,
      }}
    />
  );
});

export default CarOnMap;

