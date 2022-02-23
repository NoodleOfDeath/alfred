// @flow
import React from "react";
import Card from "@mui/material/Card";
import CardActions from "@mui/material/CardActions";
import CardContent from "@mui/material/CardContent";

function Windshield(props: any) {
  return (
    <Card
      sx={{
        minHeight: 300,
        backgroundColor: "#888",
      }}
    >
      <CardContent
        sx={{
          textAlign: "center",
        }}
      >
        This card will evenutally display the video stream of a stereoscopic
        camera and/or RPLidar module.
      </CardContent>
    </Card>
  );
}

export default Windshield;
