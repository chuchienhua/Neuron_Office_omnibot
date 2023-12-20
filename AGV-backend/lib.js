import moment from 'moment';
export const getNowDatetimeString = () => moment().format('YYYY-MM-DDTHH:mm:ss.SSSZ');