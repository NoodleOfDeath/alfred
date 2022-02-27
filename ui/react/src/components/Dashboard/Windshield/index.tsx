// @flow
import { Typography } from "@mui/material";
import Card from "@mui/material/Card";
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
        <Typography>
          This card will evenutally display the video stream of a stereoscopic
          camera and/or RPLidar module.
        </Typography>
      </CardContent>
    </Card>
  );
}

export default Windshield;
