import React, { useState, useCallback } from "react";
import Button from "react-bootstrap/Button";
import Container from "react-bootstrap/Container";
import Form from "react-bootstrap/Form";
import Nav from "react-bootstrap/Nav";
import Navbar from "react-bootstrap/Navbar";
import NavDropdown from "react-bootstrap/NavDropdown";
import { useNavigate } from "react-router-dom";
import { useSelector, useDispatch } from "react-redux";
import "./TopBar.css";
function TopBar() {
  const navigate = useNavigate();
  const dispatch = useDispatch();
  const [Tokenbool, setTokenbool] = useState(false);
  const token = useSelector((state) => state.token);
  const user = useSelector((state) => state.user);

  React.useEffect(() => {
    if (token) {
      setTokenbool(false);
    } else {
      setTokenbool(true);
    }
  }, [token]);

  const LogoutHandle = useCallback(() => {
    dispatch({ type: "LOGOUT" });
    // localStorage.removeItem("user");
    navigate("/loginoracle");
  }, [dispatch, navigate]);

  return (
    <Navbar
      expand="xl"
      sticky="top"
      className="Topbar-background"
    >
      <Container fluid>
        <Navbar.Brand>登入系統</Navbar.Brand>
        <Navbar.Toggle aria-controls="navbarScroll" />
        <Navbar.Collapse id="navbarScroll">
          {Tokenbool && (
            <Nav
              className="me-auto my-2 my-lg-0"
              style={{ maxHeight: "100px" }}
              navbarScroll
            >
              <Nav.Link
                onClick={() => {
                  navigate("/loginoracle");
                }}
              >
                Oracle System
              </Nav.Link>
            </Nav>
          )}
          {!Tokenbool && (
            <Nav
              className=" my-2 my-lg-0"
              style={{ maxHeight: "100px" }}
              navbarScroll
            >
              <NavDropdown
                title="Information"
                id="navbarScrollingDropdown"
                style={{ textAlign: "center" }}
              >
                <NavDropdown.Item
                  onClick={() => {
                    navigate("/LoginHome");
                  }}
                >
                  Home
                </NavDropdown.Item>
                <NavDropdown.Item>
                  {user ? `User: ${user.NAME}` : "Loading user..."}
                </NavDropdown.Item>
                <NavDropdown.Divider />
                <NavDropdown.Item onClick={LogoutHandle}>
                  Logout
                </NavDropdown.Item>
              </NavDropdown>
            </Nav>
          )}

          <Form className="d-flex">
            <Form.Control
              type="search"
              placeholder="Search"
              className="me-2"
              aria-label="Search"
            />
            <Button variant="outline-success">Search</Button>
          </Form>
        </Navbar.Collapse>
      </Container>
    </Navbar>
  );
}

export default TopBar;

// `SELECT * FROM PRO_MODULE_SET t1 INNER JOIN
//     (SELECT SET_NAME, MAX(VERSION) AS MaxVersion ,TITLE, DEPT
//     FROM
//         PRO_MODULE_SET
//     WHERE
//         DEPT = :DEPT AND PRD_HPD3 = :PRD_HPD3
//     GROUP BY
//        DEPT,SET_NAME,TITLE) t2
//     ON
//     t1.DEPT = t2.DEPT
//     AND t1.SET_NAME = t2.SET_NAME
//     AND t1.TITLE = t2.TITLE
//     AND t1.VERSION = t2.MaxVersion; `

// `SELECT
//     t1.SILO, t1.LINE, COALESCE(t2.cnt+1, 1) AS RANK ,
//     :LOT_NO, t1.PRD_PC, p.PCK_KIND AS PRO_WT,
//      p.PCK_KIND AS PCK_KIND, PCK_TIME AS SPEND_TIME,
//     :STR_P_PCK_TIME, t1.COMPANY, t1.FIRM,:DEPT, t1.PRD_HPD5,
//     :SPEC,t1.NOTEM, t1.UKEY, '1' AS STATUS ,
//     :USER_ID AS CREATOR ,
//     CASE
//         WHEN p.SET_NAME IS NOT NULL
//         THEN 1
//         ELSE 0
//         END
//         AS PCK_QTY ,
//     COALESCE(t2.cnt+1, 1) AS ITEM ,
//     1 AS PCK_ITEM ,
//     P.SET_NAME,:USER_ID
// FROM
//     (SELECT * FROM PRO_SCHEDULE WHERE LOT_NO = :LOT_NO AND DEPT = :DEPT AND PRD_HPD5 = :PRD_HPD5) t1
// LEFT JOIN
//     (SELECT COUNT(*) AS cnt, UKEY FROM PRO_SCHEDULE_PCK WHERE DEPT = :DEPT AND PRD_HPD5 = :PRD_HPD5 GROUP BY UKEY) t2
// ON
//     t1.UKEY = t2.UKEY
// LEFT JOIN
//     PRO_PACKAGE p
// ON
//     p.DEPT = :DEPT AND p.PRD_HPD5 = :PRD_HPD5 AND p.PCK_BAG = :SPEC AND t1.PRD_PC = p.PRD_PC `;
