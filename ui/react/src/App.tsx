// @flow
import Container from "@mui/material/Container";
import Header from "./components/Header";
import Dashboard from "./pages/Dashboard";
import { ThemeProvider } from "@mui/material/styles";

import { DarkTheme, sections } from "./types/constants";

function App() {
  return (
    <ThemeProvider theme={DarkTheme}>
      <Container maxWidth="xl" disableGutters={true}>
        <Header title="The Batcomputer" />
        <Dashboard sections={sections} />
      </Container>
    </ThemeProvider>
  );
}

export default App;
