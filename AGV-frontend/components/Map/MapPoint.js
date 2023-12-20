// MapPoint.js
import React, { memo } from "react";

const MapPoint = memo(
  ({ name, id, x, y, yaw, mapX, mapY, onEnter, onLeave, onDown }) => {
    const handleMouseEnter = () => {
      if (onEnter) {
        onEnter({ name, id, x, y, yaw });
      }
    };

    const handleMouseLeave = () => {
      if (onLeave) {
        onLeave({ name, id, x, y, yaw });
      }
    };

    const handlePointClick = () => {
      if (onDown) {
        onDown({ name, id, x, y, yaw });
      }
    };

    return (
      <div
        onMouseEnter={handleMouseEnter}
        onMouseLeave={handleMouseLeave}
        onMouseDown={handlePointClick}
        style={{
          position: "absolute",
          top: `${mapY}`, // y %
          left: `${mapX}`, // x %
          width: "12px",
          height: "12px",
          backgroundColor: "orange",
          transform: "translate(-50%, -50%)",
          borderRadius: "50%",
          cursor: "pointer",
          zIndex: 1,
        }}
      />
    );
  }
);

export default MapPoint;
