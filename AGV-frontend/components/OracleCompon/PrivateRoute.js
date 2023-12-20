import React from "react";
import { useSelector } from "react-redux";
import { Navigate } from "react-router-dom";
import PropTypes from "prop-types";

const PrivateRoute = ({ login = true, children }) => {
  const path = children.props.path || "";

  const user = useSelector((state) => state.user); //Global State
  // let authRoute = useSelector(state => state.authRoute);
  // if (authRoute) {
  //     authRoute = authRoute.map(route => '/' + route.ROUTE);
  // }

  if (login) {
    if (user) {
      // return (authRoute.includes(path)) ? children : <Navigate to="/" />;
      return children;
    } else {
      return <Navigate to={"/loginoracle"} />;
    }
  } else {
    return !user ? children : <Navigate to="/LoginHome" />;
  }
};

PrivateRoute.propTypes = {
  login: PropTypes.bool,
  children: PropTypes.any,
};

export default PrivateRoute;
