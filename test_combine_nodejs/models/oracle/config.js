const fs = require('fs');
const path = require('path');

const keyPath = path.join(__dirname, '../../SSO_TOKEN.key');
const secretKey = fs.readFileSync(keyPath).toString();

const config = {
    secret: secretKey,
    ComeFrom: 1010000000000000,
    NAME: 'kh_pbtc',
    HTTP_PORT: 10010,
    ORACLE_TNS: 'MIS_TEST.CCP.COM.TW',
    ORACLE_USERNAME: 'AC',
    ORACLE_PASSWORD: 't5001855',
    ORACLE_CONFIG: {},
};

config.ORACLE_CONFIG = {
    user: config.ORACLE_USERNAME,
    password: config.ORACLE_PASSWORD,
    connectString: config.ORACLE_TNS,
};

module.exports = config;
