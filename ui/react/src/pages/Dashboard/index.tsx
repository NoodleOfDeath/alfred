// @flow
import { Grid } from "@mui/material";
import Card from "@mui/material/Card";
import CardContent from "@mui/material/CardContent";
import React from "react";
import { DashboardProps, Unwrap } from "types/constants";
import { Windshield, Section } from "../../components/Dashboard";

class Dashboard extends React.Component<DashboardProps, DashboardProps> {
  constructor(props: DashboardProps) {
    super(props);
    this.state = {
      sections: props.sections,
    };
    const sections = [];
    for (let section of Object.values(this.state.sections)) {
    }
  }

  componentDidMount() {
    const subscriptions: {
      [key: string]: ((
        component: React.Component<DashboardProps, DashboardProps>,
        newValue: any
      ) => void)[];
    } = {};
    const topics = new Set<string>();
    for (let section of Object.values(this.state.sections)) {
      for (let module of Object.values(section.modules)) {
        for (let field of Object.values(module.fields)) {
          if (field.subscribe) {
            for (let topic of field.subscribe) {
              const subs = subscriptions[topic] || [];
              if (field.onUpdate) subs.push(field.onUpdate);
              subscriptions[topic] = subs;
              topics.add(topic);
            }
          }
        }
      }
    }
    const subscribe = () => {
      fetch(`http://batcomputer:8000/status/`, {
        method: "GET",
        mode: "cors",
        headers: {
          "Content-Type": "application/json",
        },
      })
        .then(async (response: Response) => {
          const data = JSON.parse(await response.json()) as {
            [key: string]: any;
          };
          for (let topic of Array.from(topics.values())) {
            const subs = subscriptions[topic];
            if (!subs) continue;
            for (let subscriber of subs) subscriber(this, data[topic]);
          }
        })
        .catch((error: Error) => {
          console.error(error);
        });
    };
    const subscribe_service = setInterval(subscribe, 2000);
    subscribe();
  }

  render() {
    const sections = [];
    for (let [key, section] of Object.entries(this.state.sections)) {
      sections.push(
        <Grid
          item
          key={sections.length}
          xs={Unwrap(this.state.sections[key].xs, section)}
          sm={Unwrap(this.state.sections[key].sm, section)}
          md={Unwrap(this.state.sections[key].md, section)}
          lg={Unwrap(this.state.sections[key].lg, section)}
        >
          <Section {...section} />
        </Grid>
      );
    }

    return (
      <Card>
        <CardContent>
          <Windshield />
          <Grid container spacing={2} mt={2}>
            {sections}
          </Grid>
        </CardContent>
      </Card>
    );
  }
}

export default Dashboard;
