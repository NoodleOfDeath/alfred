// @flow
import React from "react";
import Card from "@mui/material/Card";
import CardActions from "@mui/material/CardActions";
import CardContent from "@mui/material/CardContent";
import { Grid } from "@mui/material";
import Instrument from "./Instrument";

function Instruments(props: any) {
  var rows = [];
  const instruments = props.instruments;
  if (instruments && Object.keys(instruments).length > 0) {
    for (let key of Object.keys(instruments)) {
      const instrument = instruments[key];
      rows.push(
        <Grid item xs={6} key={key}>
          <Instrument {...instrument} />
        </Grid>
      );
    }
  }

  return (
    <Card
      sx={{
        minHeight: 400,
        height: "calc(50vh)",
      }}
    >
      <CardContent>
        <Grid container spacing={2}>
          {rows}
        </Grid>
      </CardContent>
    </Card>
  );
}

export default Instruments;
