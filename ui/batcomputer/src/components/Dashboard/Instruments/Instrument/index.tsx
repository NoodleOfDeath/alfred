// @flow
import React from "react";
import Icon from "@mdi/react";
import Card from "@mui/material/Card";
import CardContent from "@mui/material/CardContent";
import {
  Stack,
  Grid,
  Typography,
  TextField,
  Slider,
  Button,
  Box,
} from "@mui/material";
import { EFormFieldType } from "../../../../types/contants";

function Instrument(props: any) {
  var elems = [];
  if (props.name) {
    elems.push(
      <Grid item xs={12} key={elems.length}>
        <Typography
          variant="h6"
          component="div"
          sx={{ flexGrow: 1, textAlign: "center", marginBottom: "20px" }}
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
      var item = (
        <TextField
          label={field.label}
          variant="outlined"
          key={fields.length}
        ></TextField>
      );
      switch (field.type) {
        case EFormFieldType.slider:
          const stack = [];
          if (field.leftIcon) {
            stack.push(
              <Icon path={field.leftIcon} size={2} key={stack.length} />
            );
          }
          stack.push(
            <Slider
              aria-label={field.label}
              defaultValue={field.value}
              valueLabelDisplay="auto"
              step={0.05}
              marks
              min={field.min}
              max={field.max}
              key={stack.length}
              onChange={field.onChange || (() => {})}
            ></Slider>
          );
          if (field.rightIcon) {
            stack.push(
              <Icon path={field.rightIcon} size={2} key={stack.length} />
            );
          }
          item = (
            <Stack
              spacing={2}
              direction="row"
              sx={{ mb: 1 }}
              alignItems="center"
              key={fields.length}
            >
              {stack}
            </Stack>
          );
          break;
        case EFormFieldType.button:
          item = (
            <Button
              key={fields.length}
              variant="outlined"
              onMouseDown={field.onMouseDown || (() => {})}
              onMouseUp={field.onMouseUp || (() => {})}
            >
              {field.label}
            </Button>
          );
          break;
      }
      fields.push(item);
    }
    elems.push(
      <Box
        sx={{ width: "90%", textAlign: "center" }}
        alignItems="center"
        key={elems.length}
      >
        {fields}
      </Box>
    );
  }
  return (
    <Card
      sx={{
        border: "2px solid #000",
        backgroundColor: "secondary",
      }}
    >
      <CardContent>
        <Stack spacing={2} alignItems="center">
          {elems}
        </Stack>
      </CardContent>
    </Card>
  );
}

export default Instrument;
