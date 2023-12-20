import React from "react";
import ReactDOM from "react-dom/client";
import "./index.css";
import App from "./App";
import { Provider } from "react-redux";
import { PersistGate } from "redux-persist/integration/react";
import storagestate from "./components/storage/storagestate";
import "bootstrap/dist/css/bootstrap.min.css";
import "handsontable/dist/handsontable.full.css";

import "react-toastify/dist/ReactToastify.css"; // 引入 toastify 的 CSS
export const { persistor, store } = storagestate();

const root = ReactDOM.createRoot(document.getElementById("root"));
root.render(
  <Provider store={store}>
    <PersistGate loading={<div>Loading...</div>} persistor={persistor}>
      <App />
    </PersistGate>
  </Provider>
);
