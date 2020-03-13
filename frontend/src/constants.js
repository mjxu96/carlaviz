// Copyright (c) 2019 Uber Technologies, Inc.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
import { CarMesh } from "streetscape.gl";

export const UI_THEME = {
  extends: "dark",
  background: "rgba(51,51,51,0.9)",
  backgroundAlt: "#222222",

  controlColorPrimary: "#858586",
  controlColorSecondary: "#636364",
  controlColorDisabled: "#404042",
  controlColorHovered: "#F8F8F9",
  controlColorActive: "#5B91F4",

  textColorPrimary: "#F8F8F9",
  textColorSecondary: "#D0D0D1",
  textColorDisabled: "#717172",
  textColorInvert: "#1B1B1C",

  fontFamily:
    '-apple-system, BlinkMacSystemFont, "Segoe UI", Arial, sans-serif',
  fontSize: 14,
  fontWeight: 200,

  shadow: "0 2px 4px 0 rgba(0, 0, 0, 0.15)"
};

/* eslint-disable camelcase */

export const CAR = CarMesh.sedan({
  origin: [1.08, 0, 0],
  length: 4.7,
  width: 2.5,
  height: 1.7,
  color: [128, 128, 128]
});

export const APP_SETTINGS = {
  viewMode: {
    type: "select",
    title: "View Mode",
    data: { TOP_DOWN: "Top Down", PERSPECTIVE: "Perspective", DRIVER: "Driver" }
  },
  showTooltip: {
    type: "toggle",
    title: "Show Tooltip"
  }
};

export const XVIZ_STYLE = {
  "/object/vehicles": [
    { name: "selected", style: { fill_color: "#ff8000aa" } }
  ],
  "/object/walkers": [{ name: "selected", style: { fill_color: "#ff8000aa" } }],
  "/lidar/points": [{ style: { point_color_mode: "ELEVATION" } }]
};
