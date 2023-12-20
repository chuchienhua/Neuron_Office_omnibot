import axios from "axios";
const getURL = (api, params = []) => {
  // let url = "http://192.168.102.38:3003/" + api;
  let url = "http://192.168.8.241:3561/" + api;
  params.forEach((element) => {
    url += "/" + element.toString();
  });
  return url;
};

//從LocalStorage取出persist
function parsePersistFromLocalStorage() {
  let persist = localStorage.getItem("persist:root");
  let obj = null;
  if (persist) {
    obj = JSON.parse(persist);
    if (obj) {
      for (let key in obj) {
        obj[key] = JSON.parse(obj[key]);
      }
    }
  }
  return obj;
}

//從localStorage中取出token
function getTokenFromLocalStorage() {
  let obj = parsePersistFromLocalStorage();
  if (obj) {
    return obj.token;
  }

  return null;
}

//從localStorage中取出firm
function getFirmFromLocalStorage() {
  let obj = parsePersistFromLocalStorage();
  if (obj) {
    return obj.firm;
  }

  return null;
}

//統一API用的axios config
const pbtAxiosConfig = {
  transformRequest: axios.defaults.transformRequest.concat((data, headers) => {
    headers["Authorization"] = `Bearer ${getTokenFromLocalStorage()}`;
    headers["Firm"] = `${getFirmFromLocalStorage()}`;
    return data;
  }),
  timeout: 60000,
};

export default {
  getURL,
  pbtAxiosConfig,
  getTokenFromLocalStorage,
  getFirmFromLocalStorage,
};
