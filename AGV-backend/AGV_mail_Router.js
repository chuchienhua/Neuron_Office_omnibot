// Description: Router for mail
// const Mailer = require('./mail.js');
import Mailer from './mail.js';
// const express = require('express');
import express from 'express';

const AgvMailRouter = express.Router()

AgvMailRouter.post('/sendmail', (req, res) => {
    const user = req.body.user;
    const carid = req.body.carid;
    console.log(user);
    console.log(carid);
    Mailer.AGV_Stop_problem(user, carid).then(result =>
        res.json(result.data)
        // console.log(result.data)
    );
})

export default AgvMailRouter;
// module.exports = AgvMailRouter;