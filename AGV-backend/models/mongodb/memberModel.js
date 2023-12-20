// 會員資料庫溝通模組檔
import mongoose from "mongoose";
var db = mongoose.connection;
async function connectDB() {
  try {
    const client = await mongoose.connect(
      "mongodb://PBF_CCD:cw5pg2o0e68g@203.69.135.73:27017/ML_PBF_CCD?authSource=admin&authMechanism=SCRAM-SHA-256",
      { useNewUrlParser: true, useUnifiedTopology: true }
    );
  } catch (error) {
    console.error("Database connection failed: ", error);
  }
}
connectDB();
db.on("error", console.error.bind(console, "CONNECTION ERROR!!"));
var newmemberSchema = new mongoose.Schema({
  status: String,
  time: Date,
  pointlist:[{
    site: Number,
    x: Number,
    y: Number,
    yaw: Number,
  }],
});
newmemberSchema.set("Collation", "TestCar");
var model = mongoose.model("TestCar", newmemberSchema, "TestCar");

export default model;
