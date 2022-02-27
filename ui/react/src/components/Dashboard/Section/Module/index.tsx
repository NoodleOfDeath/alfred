// @flow
import Card from "@mui/material/Card";
import CardContent from "@mui/material/CardContent";
import { Stack, Grid, Typography, Box, Button } from "@mui/material";
import Widget from "./Widget";
import { TModuleProps } from "types/constants";

function Module(props: TModuleProps) {
  var stack = [];
  if (props.name) {
    stack.push(
      <Grid item key={stack.length} xs={12}>
        <Typography
          variant="h5"
          component="div"
          sx={{ fontWeight: "bold", flexGrow: 1, textAlign: "center" }}
        >
          {props.name}
        </Typography>
      </Grid>
    );
  }
  if (props.fields && Object.keys(props.fields).length > 0) {
    const fields = [];
    for (let [key, field] of Object.entries(
      props.fields as { [key: string]: any }
    )) {
      fields.push(<Widget key={fields.length} {...field} />);
    }
    stack.push(
      <Grid
        container
        key={stack.length}
        sx={{ width: "90%", textAlign: "center" }}
        alignItems="center"
        spacing={2}
      >
        {fields}
      </Grid>
    );
  }
  return (
    <Card
      variant="outlined"
      sx={{
        display: "flex",
        flexGrow: 1,
        flexDirection: "column",
        height: "100%",
      }}
    >
      <CardContent>
        <Stack spacing={2} alignItems="center">
          {stack}
        </Stack>
      </CardContent>
    </Card>
  );
}

export default Module;
