function quaternionToYaw(z, w) {
    // Yaw is the rotation around the Z axis
    // atan2 returns the angle between the positive x-axis of a plane and the point given by the coordinates
    let yaw = Math.atan2(2.0 * z * w, 1.0 - 2.0 * z * z);
    return Number(yaw.toFixed(6)); // The angle is in radians
}

export default quaternionToYaw;