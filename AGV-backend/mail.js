// const moment = require('moment');
import moment from 'moment';
// const fs = require('fs');
import fs from 'fs';
// const FormData = require('form-data');
import FormData from 'form-data';
// const axios = require('axios');
import axios from 'axios';
// const XLSL = require('xlsx-js-style');


const getNowDatetimeString = () => {
    return moment().format('YYYY-MM-DD HH:mm:ss');
}

const Send_Mail_URL = "http://192.168.8.104:3501/sendmail";

const sendMail = (to, subject, html, cc, bcc, attachment = [], from = "AGV_Problem_Info <AGV_IOT@ccpgp.com>") => {
    let axiosConfig = { proxy: false };
    const formData = new FormData();
    formData.append('to', to);
    formData.append('subject', subject);
    formData.append('html', html);
    formData.append('cc', cc || "");
    formData.append('bcc', bcc || "");
    formData.append('from', from);

    if (attachment.length) {
        axiosConfig["Content-Type"] = "multipart/form-data";
        attachment.forEach((attachment) => {
            formData.append('attachment[]', fs.createReadStream(attachment));
        });
    } else {
        axiosConfig["Content-Type"] = "application/x-www-form-urlencoded";
    }
    return axios.post(Send_Mail_URL, formData, axiosConfig);
}

export const AGV_Stop_problem = async (user, carid) => {
    const mailuser = user;
    const mailcarid = carid;
    const to = "朱建樺 <chienhua_chu@ccpgp.com>"
    const subject = "AGV Stop Problem";
    const html = `        
        <h2>AGV連線異常問題</h2>
        <p>使用者<code>${mailuser} </code>於AGV<code>${mailcarid}</code>派定任務實因連線中斷
            ,車子已停止運行,請盡速檢查連線</p>
    `;
    const cc = "";
    const bcc = "";
    return sendMail(to, subject, html, cc, bcc);
}

export default {AGV_Stop_problem};
