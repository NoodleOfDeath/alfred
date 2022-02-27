// @flow
import Card from "@mui/material/Card";
import CardContent from "@mui/material/CardContent";
import { Box, Grid, Typography } from "@mui/material";
import Module from "./Module";
import { Unwrap, TModuleSectionProps } from "types/constants";

function Section(props: TModuleSectionProps) {
  var items = [];
  const modules = props.modules;
  if (modules && Object.keys(modules).length > 0) {
    for (let key of Object.keys(modules)) {
      const module = modules[key];
      items.push(
        <Grid
          item
          key={key}
          xs={Unwrap(modules[key].xs, module)}
          sm={Unwrap(modules[key].sm, module)}
          md={Unwrap(modules[key].md, module)}
          lg={Unwrap(modules[key].lg, module)}
        >
          <Module {...module} />
        </Grid>
      );
    }
  }
  return (
    <Card variant="outlined">
      <CardContent>
        <Box>
          <Typography
            variant="h4"
            sx={{
              marginBottom: 3,
            }}
          >
            {props.name}
          </Typography>
        </Box>
        <Grid container>{items}</Grid>
      </CardContent>
    </Card>
  );
}

export default Section;
