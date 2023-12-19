const express = require('express');
const memberModel = require("../models/mongodb/memberModel.js");
const AGV_mongodb = express.Router();

//車子派車資料
AGV_mongodb.post("/car_pointlist", function (req, res) {
  console.log(req.body);
  var newmember = new memberModel({
    status: req.body.status,
    time: req.body.time,
    pointlist:req.body.pointlist,
  });
  newmember.save(function (err, data) {
    if (err) {
      res.json({ status: 1, msg: "error" });
    } else {
      res.json({ status: 0, msg: "success", data: data });
    }
  });
});

AGV_mongodb.get("/get_car_pointlist", async (req, res) => {
  try {
    const alldata = await memberModel.find();
    res.json(alldata);
    console.log(alldata);
  } catch (err) {
    res.status(500).json({ message: err.message });
  }
});

module.exports = AGV_mongodb;