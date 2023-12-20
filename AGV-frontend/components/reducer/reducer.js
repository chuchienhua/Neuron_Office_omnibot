const LOGIN_SUCCESS = "LOGIN_SUCCESS";
const LOGIN_FAILURE = "LOGIN_FAILURE";
const LOGOUT = "LOGOUT";

const initialState = {
  user: undefined,
  token: undefined,
  authRoute: undefined,
  firm: undefined,
};

const userReducer = (state = initialState, action) => {
  switch (action.type) {
    case LOGIN_SUCCESS:
      return {
        // ...state,
        user: action.payload.user,
        token: action.payload.token,
        authRoute: action.payload.authRoute,
        firm: action.payload.firm,
      };
    case LOGIN_FAILURE:
      return {
        // ...state,
        user: undefined,
        token: undefined,
        authRoute: undefined,
        firm: undefined,
      };
    case LOGOUT:
      return {
        // ...state,
        user: undefined,
        token: undefined,
        authRoute: undefined,
        firm: undefined,
      };
    default:
      return state;
  }
};

export default userReducer;
