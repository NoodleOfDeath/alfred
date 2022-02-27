import { createTheme } from "@mui/material/styles";
import {
  mdiTortoise,
  mdiRabbit,
  mdiRestore,
  mdiReload,
  mdiBattery60,
  mdiBattery,
  mdiBattery20,
  mdiBattery80,
  mdiBattery40,
  mdiFire,
  mdiFlash,
  mdiLightningBoltOutline,
  mdiThermometer,
  mdiArrowDownBold,
  mdiArrowUpBold,
} from "@mdi/js";
import React from "react";

export const APIEndpoint = "http://batcomputer:8000";

/**
 *
 */
export const DarkTheme = createTheme({
  palette: {
    mode: "dark",
    background: {
      default: "#000",
      paper: "#000",
    },
    text: {
      primary: "#FFF",
    },
    primary: {
      main: "#D27519",
    },
    secondary: {
      main: "#1976D2",
    },
  },
  typography: {
    fontFamily: [
      "DM Mono",
      "-apple-system",
      "BlinkMacSystemFont",
      '"Segoe UI"',
      "Roboto",
      '"Helvetica Neue"',
      "Arial",
      "sans-serif",
      '"Apple Color Emoji"',
      '"Segoe UI Emoji"',
      '"Segoe UI Symbol"',
    ].join(","),
  },
});

/**
 * Simple state management without using REDUX for simplicity.
 */
export const store = {
  battery: NaN,
  mousedown: false,
  driveSpeed: 0.0,
  rotateSpeed: 0.0,
};

/**
 *
 */
export type WrappedValue<T, Props> =
  | T
  | ((props?: Props, rtValue?: T | undefined) => T | undefined);

/**
 *
 */
export function Unwrap<T, Props>(
  value?: WrappedValue<T, Props>,
  props?: Props,
  rtValue?: T
): T | undefined {
  if (typeof value === "function" && props)
    return (value as (props: Props, rtValue: T | undefined) => T | undefined)(
      props,
      rtValue
    );
  return value as T;
}

/**
 *
 */
export type TModuleSectionProps = {
  name?: string;
  xs?: WrappedValue<number, TModuleSectionProps>;
  sm?: WrappedValue<number, TModuleSectionProps>;
  md?: WrappedValue<number, TModuleSectionProps>;
  lg?: WrappedValue<number, TModuleSectionProps>;
  modules: { [key: string]: TModuleProps };
};

/**
 *
 */
export type TModuleProps = {
  name: string;
  xs?: WrappedValue<number, TModuleProps>;
  sm?: WrappedValue<number, TModuleProps>;
  md?: WrappedValue<number, TModuleProps>;
  lg?: WrappedValue<number, TModuleProps>;
  fields: {
    [key: string]: TWidgetProps;
  };
};
/**
 *
 */
export enum EWidgetType {
  text,
  slider,
  button,
  label,
}

/**
 *
 */
export type TWidgetProps = {
  type: EWidgetType;
  label?: WrappedValue<string, TWidgetProps>;
  subscribe?: string[];
  value?: WrappedValue<any, TWidgetProps>;
  defaultValue?: WrappedValue<any, TWidgetProps>;
  displayValue?: WrappedValue<any, TWidgetProps>;
  onReleaseDefaultValue?: WrappedValue<boolean, TWidgetProps>;
  steps?: WrappedValue<number, TWidgetProps>;
  marks?: WrappedValue<boolean, TWidgetProps>;
  min?: WrappedValue<number, TWidgetProps>;
  max?: WrappedValue<number, TWidgetProps>;
  startIcon?: WrappedValue<string, TWidgetProps>;
  endIcon?: WrappedValue<string, TWidgetProps>;
  xs?: WrappedValue<number, TWidgetProps>;
  sm?: WrappedValue<number, TWidgetProps>;
  md?: WrappedValue<number, TWidgetProps>;
  lg?: WrappedValue<number, TWidgetProps>;
  onChange?: (
    event: Event,
    newValue: number | number[],
    activeThumb: number
  ) => void;
  onClick?: (event: React.MouseEvent<HTMLElement>) => void;
  onMouseDown?: (event: React.MouseEvent<HTMLElement>) => void;
  onMouseUp?: (event: React.MouseEvent<HTMLElement>) => void;
  onMouseEnter?: (event: React.MouseEvent<HTMLElement>) => void;
  onMouseOut?: (event: React.MouseEvent<HTMLElement>) => void;
  onMouseLeave?: (event: React.MouseEvent<HTMLElement>) => void;
  onUpdate?: (
    component?: React.Component<DashboardProps, DashboardProps>,
    newValue?: any
  ) => void;
};

export type DashboardProps = {
  sections: { [key: string]: TModuleSectionProps };
};

const sendRequest = (
  url: string,
  body: { [key: string]: any },
  repeat: () => boolean
) => {
  if (!repeat()) return;
  fetch(url, {
    method: "POST",
    mode: "cors",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify(body),
  })
    .then((response: Response) => {})
    .catch((error: Error) => {
      console.error(error);
    })
    .finally(() => {
      if (repeat()) sendRequest(url, body, repeat);
    });
};

window.addEventListener("mouseup", function (event) {
  store.mousedown = false;
});

/**
 *
 */
export var sections: { [key: string]: TModuleSectionProps } = {
  control: {
    name: "Controls",
    xs: 12,
    sm: 12,
    md: 12,
    lg: 6,
    modules: {
      speed: {
        name: "Control Speeds",
        fields: {
          driveSpeed: {
            type: EWidgetType.slider,
            label: "Speed m/s",
            value: store.driveSpeed,
            defaultValue: 0.0,
            onReleaseDefaultValue: true,
            displayValue: (props?: TWidgetProps, rtValue?: any): any => {
              const value = rtValue ? rtValue : props ? props.value : undefined;
              return `${(value as number).toFixed(1)} m/s`;
            },
            steps: 0.1,
            min: -0.5,
            max: 0.5,
            startIcon: mdiArrowDownBold,
            endIcon: mdiArrowUpBold,
            onChange: (
              event: Event,
              newValue: number | number[],
              activeThumb: number
            ) => {
              store.mousedown = true;
              sendRequest(
                `${APIEndpoint}/drive/`,
                { velocity: newValue as number },
                () => store.mousedown
              );
            },
          },
          rotateSpeed: {
            type: EWidgetType.slider,
            label: "Speed rad/s",
            value: store.rotateSpeed,
            defaultValue: 0.0,
            onReleaseDefaultValue: true,
            displayValue: (props?: TWidgetProps, rtValue?: any): any => {
              const value = rtValue ? rtValue : props ? props.value : undefined;
              return `${(value as number).toFixed(2)} rad/s`;
            },
            steps: 0.25,
            min: -4.25,
            max: 4.25,
            startIcon: mdiRestore,
            endIcon: mdiReload,
            onChange: (
              event: Event,
              newValue: number | number[],
              activeThumb: number
            ) => {
              store.mousedown = true;
              sendRequest(
                `${APIEndpoint}/rotate/`,
                { velocity: newValue as number },
                () => store.mousedown
              );
            },
          },
        },
      },
    },
  },
  sensors: {
    name: "Sensors",
    xs: 12,
    sm: 12,
    md: 12,
    lg: 6,
    modules: {
      battery: {
        name: "iRobot Create 2 Battery",
        fields: {
          charge_ratio: {
            type: EWidgetType.label,
            label: "Charge",
            xs: 12,
            sm: 6,
            md: 6,
            lg: 6,
            displayValue: (props?: TWidgetProps, rtValue?: any): any => {
              const value = rtValue ? rtValue : props ? props.value : undefined;
              return `${((value as number) * 100).toFixed(2)}%`;
            },
            startIcon: (props?: TWidgetProps, rtValue?: any): string => {
              const value = rtValue ? rtValue : props ? props.value : undefined;
              if (value >= 0.0 && value <= 0.2) {
                return mdiBattery20;
              } else if (value > 0.2 && value <= 0.4) {
                return mdiBattery40;
              } else if (value > 0.4 && value <= 0.6) {
                return mdiBattery60;
              } else if (value > 0.6 && value <= 0.8) {
                return mdiBattery80;
              }
              return mdiBattery;
            },
            subscribe: ["/battery/charge_ratio"],
            onUpdate: (
              component?: React.Component<DashboardProps, DashboardProps>,
              newValue?: any
            ) => {
              sections.sensors.modules.battery.fields.charge_ratio.value =
                newValue as number;
              if (component) component.setState({ sections: sections });
            },
          },
          temperature: {
            type: EWidgetType.label,
            label: "Temperature",
            xs: 12,
            sm: 6,
            md: 6,
            lg: 6,
            displayValue: (props?: TWidgetProps, rtValue?: any): any => {
              const value = rtValue ? rtValue : props ? props.value : undefined;
              return `${value as number}°C`;
            },
            startIcon: (props?: TWidgetProps, rtValue?: any): string => {
              const value = rtValue ? rtValue : props ? props.value : undefined;
              if (value > 50) {
                return mdiFire;
              }
              return mdiThermometer;
            },
            subscribe: ["/battery/temperature"],
            onUpdate: (
              component?: React.Component<DashboardProps, DashboardProps>,
              newValue?: any
            ) => {
              sections.sensors.modules.battery.fields.temperature.value =
                newValue as number;
              if (component) component.setState({ sections: sections });
            },
          },
          current: {
            type: EWidgetType.label,
            label: "Current",
            xs: 12,
            sm: 6,
            md: 6,
            lg: 6,
            displayValue: (props?: TWidgetProps, rtValue?: any): any => {
              const value = rtValue ? rtValue : props ? props.value : undefined;
              return `${value < 0 ? "-" : ""}${Math.abs(
                value as number
              ).toFixed(2)}A`;
            },
            startIcon: mdiFlash,
            subscribe: ["/battery/current"],
            onUpdate: (
              component?: React.Component<DashboardProps, DashboardProps>,
              newValue?: any
            ) => {
              sections.sensors.modules.battery.fields.current.value =
                newValue as number;
              if (component) component.setState({ sections: sections });
            },
          },
          voltage: {
            type: EWidgetType.label,
            label: "Voltage",
            xs: 12,
            sm: 6,
            md: 6,
            lg: 6,
            displayValue: (props?: TWidgetProps, rtValue?: any): any => {
              const value = rtValue ? rtValue : props ? props.value : undefined;
              return `${value < 0 ? "-" : ""}${Math.abs(
                value as number
              ).toFixed(2)}V`;
            },
            startIcon: mdiLightningBoltOutline,
            subscribe: ["/battery/voltage"],
            onUpdate: (
              component?: React.Component<DashboardProps, DashboardProps>,
              newValue?: any
            ) => {
              sections.sensors.modules.battery.fields.voltage.value =
                newValue as number;
              if (component) component.setState({ sections: sections });
            },
          },
        },
      },
    },
  },
};
