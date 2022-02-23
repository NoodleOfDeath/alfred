// @flow
import React from "react";
import Card from "@mui/material/Card";
import CardActions from "@mui/material/CardActions";
import CardContent from "@mui/material/CardContent";
import { Windshield, Instruments } from "../../components/Dashboard";

function Dashboard(props: any) {
  return (
    <Card>
      <CardContent>
        <Windshield />
        <Instruments instruments={props.instruments} />
      </CardContent>
    </Card>
  );
}

export default Dashboard;
