// apiFunctions.js
import axios from 'axios';
import Utils from "../../Utils.js";
export const saveCarList = (Savecarpoint) => {
    const apiurl = Utils.getURL("newmember/car_pointlist");
    axios.post(apiurl, {
        status: "Car Dispatch",
        time: Date.now(),
        pointlist: Savecarpoint,
    })
        .then((res) => {
            console.log(res.status);
        });
};

export const pathCar_cancel = () => {
    const apiurl = Utils.getURL("newmember/car_pointlist");
    axios.post(apiurl, {
        status: "Path cancel",
        time: Date.now(),
        pointlist: [],
    })
        .then((res) => {
            console.log(res.status);
        });
};
