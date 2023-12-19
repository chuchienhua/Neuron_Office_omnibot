import express from "express";
import memberModel from "../models/mongodb/memberModel.js";
var router = express.Router();

//車子派車資料
router.post("/car_pointlist", function (req, res) {
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

router.get("/get_car_pointlist", async (req, res) => {
  try {
    const alldata = await memberModel.find();
    res.json(alldata);
    console.log(alldata);
  } catch (err) {
    res.status(500).json({ message: err.message });
  }
});

export default router;
