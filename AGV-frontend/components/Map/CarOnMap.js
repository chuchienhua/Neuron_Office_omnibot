// CarOnMap.js
import React, { memo } from "react";
import carimage from "./car.png";

const CarOnMap = memo(({ mapX, mapY , yaw }) => {
  return (
    <div
      style={{
        position: "absolute",
        top: `${mapY}`, // y %
        left: `${mapX}`, // x %
        width: "32px",
        height: "32px",
   
        backgroundImage: `url(${carimage})`,
        // backgroundColor: "rgba(255, 0, 0, 0.5)",
        backgroundSize: "contain", // 确保图片按比例缩放
        backgroundRepeat: "no-repeat", // 防止图片重复
        transform: `translate(-50%, -50%) rotate(${yaw}deg)`,
        borderRadius: "50%",
        cursor: "pointer",
        pointerEvents: "none",
      }}
    />
  );
});

export default CarOnMap;
