import { createTheme } from "@mui/material/styles";
import { mdiTortoise, mdiRabbit, mdiRestore, mdiReload } from "@mdi/js";

/**
 *
 */
export enum EFormFieldType {
  text,
  slider,
  button,
}

export const APIEndpointPoint = "http://192.168.68.158:8000";

/**
 *
 */
export const DarkTheme = createTheme({
  palette: {
    mode: "dark",
    primary: {
      main: "#1976d2",
    },
  },
});

/**
 * Simple state management without using REDUX for simplicity.
 */
export const store = {
  mousedown: false,
  driveSpeed: 0.25,
  turnSpeed: 1.0,
};

/**
 *
 */
export const instruments = {
  drive: {
    name: "Drive Robot",
    fields: {
      speed: {
        label: "Speed m/s",
        type: EFormFieldType.slider,
        value: store.driveSpeed,
        min: -0.5,
        max: 0.5,
        leftIcon: mdiTortoise,
        rightIcon: mdiRabbit,
        onChange: (event: Event, newValue: number | number[]) => {
          store.driveSpeed = newValue as number;
        },
      },
      drive: {
        label: "Drive",
        type: EFormFieldType.button,
        onMouseDown: (event: MouseEvent) => {
          // Dispatch a drive request to the API
          store.mousedown = true;
          const sendRequest = () => {
            if (!store.mousedown) return;
            console.log(`sending drive request: speed = ${store.driveSpeed}`);
            fetch(`${APIEndpointPoint}/drive/`, {
              method: "POST",
              mode: "cors",
              headers: {
                "Content-Type": "application/json",
              },
              body: JSON.stringify({
                speed: store.driveSpeed,
              }),
            })
              .then((response: Response) => {})
              .catch((error: Error) => {
                console.error(error);
              })
              .finally(() => {
                if (store.mousedown) sendRequest();
              });
          };
          sendRequest();
        },
        onMouseUp: (event: MouseEvent) => {
          store.mousedown = false;
        },
      },
    },
  },
  rotate: {
    name: "Rotate Robot",
    fields: {
      speed: {
        label: "Speed rad/s",
        type: EFormFieldType.slider,
        value: store.turnSpeed,
        min: -4.25,
        max: 4.25,
        leftIcon: mdiRestore,
        rightIcon: mdiReload,
        onChange: (event: Event, newValue: number | number[]) => {
          store.turnSpeed = newValue as number;
        },
      },
      rotate: {
        label: "Rotate",
        type: EFormFieldType.button,
        onMouseDown: (event: MouseEvent) => {
          // Dispatch a turn request to the API
          store.mousedown = true;
          const sendRequest = () => {
            if (!store.mousedown) return;
            console.log(`sending turn request: speed = ${store.turnSpeed}`);
            fetch(`${APIEndpointPoint}/rotate/`, {
              method: "POST",
              mode: "cors",
              headers: {
                "Content-Type": "application/json",
              },
              body: JSON.stringify({
                speed: store.turnSpeed,
              }),
            })
              .then((response: Response) => {})
              .catch((error: Error) => {
                console.error(error);
              })
              .finally(() => {
                if (store.mousedown) sendRequest();
              });
          };
          sendRequest();
        },
        onMouseUp: (event: MouseEvent) => {
          store.mousedown = false;
        },
      },
    },
  },
};
