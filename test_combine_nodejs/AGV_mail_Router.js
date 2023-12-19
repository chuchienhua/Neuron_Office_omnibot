// Description: Router for mail
import Mailer from './mail.js';
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

// // Description: Router for mail
// const Mailer = require('./mail.js');
// const express = require('express');

// const AgvMailRouter = express.Router()

// AgvMailRouter.post('/sendmail', (req, res) => {
//     const user = req.body.user;
//     const carid = req.body.carid;
//     console.log(user);
//     console.log(carid);
//     Mailer.AGV_Stop_problem(user, carid).then(result =>
//         res.json(result.data)
//         // console.log(result.data)
//     );
// })

// module.exports = AgvMailRouter;
