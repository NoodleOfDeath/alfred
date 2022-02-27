// @flow
import Icon from "@mdi/react";
import {
  Stack,
  Box,
  Slider,
  Button,
  Grid,
  Card,
  CardContent,
  Typography,
} from "@mui/material";
import { useCallback, useState } from "react";
import {
  EWidgetType,
  TWidgetProps,
  Unwrap,
} from "../../../../../types/constants";

function Widget(props: TWidgetProps) {
  const [value, setValue] = useState(props.value);

  function onChange(
    event: Event,
    newValue: number | number[],
    activeThumb: number
  ) {
    setValue(newValue);
    if (props.onChange) props.onChange(event, newValue, activeThumb);
  }

  window.addEventListener("mouseup", function (event) {
    if (props.defaultValue !== undefined && props.onReleaseDefaultValue)
      setValue(props.defaultValue);
  });

  const stack = [];

  const startIcon = Unwrap(props.startIcon, props);
  const endIcon = Unwrap(props.endIcon, props);

  if (startIcon) {
    stack.push(<Icon key={stack.length} path={startIcon} size={2} />);
  }
  switch (props.type) {
    case EWidgetType.slider:
      stack.push(
        <Slider
          key={stack.length}
          aria-label={Unwrap(props.label, props)}
          value={value}
          valueLabelDisplay="auto"
          step={Unwrap(props.steps, props) || 1}
          marks={Unwrap(props.marks, props)}
          min={Unwrap(props.min, props)}
          max={Unwrap(props.max, props)}
          onChange={onChange}
          onMouseDown={props.onMouseDown}
          onMouseUp={props.onMouseUp}
          onMouseOut={props.onMouseOut}
          onMouseLeave={props.onMouseLeave}
        ></Slider>
      );
      break;
    case EWidgetType.button:
      stack.push(
        <Button
          key={stack.length}
          variant="contained"
          onClick={props.onClick}
          onMouseDown={props.onMouseDown}
          onMouseUp={props.onMouseUp}
          onMouseOut={props.onMouseOut}
          onMouseLeave={props.onMouseLeave}
        >
          {Unwrap(props.label, props)}
        </Button>
      );
      break;
    case EWidgetType.label:
      stack.push(
        <Box key={stack.length}>
          <Typography
            component="div"
            color="primary"
            sx={{
              fontWeight: "bold",
              flexGrow: 1,
              textAlign: "left",
            }}
          >
            {Unwrap(props.label, props)}
          </Typography>
          <Typography
            component="div"
            sx={{
              flexGrow: 1,
              textAlign: "left",
            }}
          >
            {Unwrap(props.displayValue, props, value) || value}
          </Typography>
        </Box>
      );
      break;
  }
  if (endIcon) {
    stack.push(<Icon key={stack.length} path={endIcon} size={2} />);
  }
  const content = (
    <Stack spacing={2} direction="row" alignItems="center">
      {stack}
    </Stack>
  );
  if (props.type === EWidgetType.slider) {
    stack.push(
      <Grid
        item
        key={stack.length}
        sx={{
          minWidth: 150,
        }}
      >
        <Typography>
          {Unwrap(props.displayValue, props, value) || value}
        </Typography>
      </Grid>
    );
  }
  return (
    <Grid
      item
      xs={Unwrap(props.xs, props) || 12}
      sm={Unwrap(props.sm, props)}
      md={Unwrap(props.md, props)}
      lg={Unwrap(props.lg, props)}
    >
      <Card>
        <CardContent>{content}</CardContent>
      </Card>
    </Grid>
  );
}

export default Widget;
