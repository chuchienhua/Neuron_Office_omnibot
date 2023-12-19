import axios from "axios";
import * as oracledb from "../models/oracle/oraclecon.js";
import express from "express";
import { getUserInformation } from "../gettokenuser.js";

const oracleRouter = express.Router();


export default oracleRouter;
