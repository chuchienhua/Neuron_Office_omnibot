const moment = require('moment');
const getNowDatetimeString = () => moment().format('YYYY-MM-DDTHH:mm:ss.SSSZ');

module.exports = { getNowDatetimeString };