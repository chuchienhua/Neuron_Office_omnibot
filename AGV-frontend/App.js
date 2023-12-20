import React , { lazy, Suspense }from "react";
import { BrowserRouter, Route, Routes } from "react-router-dom";
import TopBar from "./components/TopBar";
import PrivateRoute from "./components/OracleCompon/PrivateRoute.js"
import { ToastContainer } from 'react-toastify';

const LoginOracle = lazy(() => import('./components/OracleCompon/LoginOracle.js'));
const HomeOracle = lazy(() => import('./components/OracleCompon/HomeOracle.js'));
const Car_Interface = lazy(() => import('./components/OracleCompon/Car_Interface.js'));
const LoginHome = lazy(() => import('./components/OracleCompon/LoginHome.js'));

function App() {
  return (
    <BrowserRouter>
     <TopBar />
     <ToastContainer autoClose={2500} position={'top-right'} />
     <Suspense fallback={<div>Loading...</div>}>
      <Routes>
        <Route path='homeoracle' element={<PrivateRoute login={true}><HomeOracle path='/homeoracle' /></PrivateRoute>} />
        <Route path='LoginHome' element={<PrivateRoute login={true}><LoginHome path='/LoginHome' /></PrivateRoute>} />
        <Route path='car_interface' element={<PrivateRoute login={true}><Car_Interface path='/car_interface' /></PrivateRoute>} />
        <Route path='loginoracle' element={<PrivateRoute login={false}><LoginOracle/></PrivateRoute>} />
      </Routes>
      </Suspense>
    </BrowserRouter>
  );
}

export default App;
