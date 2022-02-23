// @flow
import React from "react";
import Container from "@mui/material/Container";
import Header from "./components/Header";
import Dashboard from "./pages/Dashboard";
import { ThemeProvider } from "@mui/material/styles";

import { DarkTheme, instruments } from "./types/contants";

function App() {
  return (
    <ThemeProvider theme={DarkTheme}>
      <Container className="App">
        <Header title="The Batcomputer" />
        <Dashboard instruments={instruments} />
      </Container>
    </ThemeProvider>
  );
}

export default App;
