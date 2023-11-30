// Description: 用于将欧拉角转换为四元数
function eulerToQuaternion(yaw, pitch, roll) { // yaw (Z), pitch (Y), roll (X)
  // 缩写各种角度函数
  let cy = Math.cos(yaw * 0.5);
  let sy = Math.sin(yaw * 0.5);
  let cp = Math.cos(pitch * 0.5);
  let sp = Math.sin(pitch * 0.5);
  let cr = Math.cos(roll * 0.5);
  let sr = Math.sin(roll * 0.5);

  let q = {};
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;

  // console.log('q.w:', q.w, 'q.x:', q.x, 'q.y:', q.y, 'q.z:', q.z);

  return q;
}

module.exports = eulerToQuaternion;