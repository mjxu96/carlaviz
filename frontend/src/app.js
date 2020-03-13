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

/* global document, console */
/* eslint-disable no-console, no-unused-vars, no-undef */
import React, { PureComponent } from "react";
import { render } from "react-dom";

import { GeoJsonLayer } from "@deck.gl/layers";
import { COORDINATE_SYSTEM } from "@deck.gl/core";
import {
  LogViewer,
  StreamSettingsPanel,
  MeterWidget,
  XVIZPanel,
  XVIZLiveLoader,
  VIEW_MODE
} from "streetscape.gl";
import { Form, ThemeProvider } from "@streetscape.gl/monochrome";

import { APP_SETTINGS, XVIZ_STYLE, CAR, UI_THEME } from "./constants";

const carlaLog = new XVIZLiveLoader({
  logGuid: "mock",
  bufferLength: 10,
  serverConfig: {
    defaultLogLength: 50,
    serverUrl: "ws://" + __HOST_IP__ + ":8081"
  },
  worker: true,
  maxConcurrency: 10
});

const tableComponentProps = {
  table: {
    height: 120
  }
};
class CarlaViz extends PureComponent {
  state = {
    log: carlaLog,
    settings: {
      viewMode: "PERSPECTIVE",
      showTooltip: true
    }
  };

  componentDidMount() {
    const { log } = this.state;
    log
      .on("ready", () => {
        const metadata = log.getMetadata();
        if (metadata.map) {
          const mapLayer = new GeoJsonLayer({
            coordinateSystem: COORDINATE_SYSTEM.METER_OFFSETS,
            coordinateOrigin: [0, 0, 0],
            id: "carla_map",
            data: metadata.map,
            stroked: true,
            filled: true,
            wireframe: true,
            extruded: true,
            getFillColor: [255, 255, 255, 255],
            getLineColor: [255, 255, 255, 255],
            getLineWidth: 0.1,
            getRadius: 0.00001,
            opacity: 10
          });
          this.setState({ map: mapLayer });
          console.log("get map");
        } else {
          console.log("receive metadata without map");
        }
      })
      .on("error", console.error)
      .connect();
  }

  _onSettingsChange = changedSettings => {
    this.setState({
      settings: { ...this.state.settings, ...changedSettings }
    });
  };

  render() {
    const { log, map, settings } = this.state;
    const customLayers = [map];

    return (
      <div id="container">
        <div id="control-panel">
          <XVIZPanel log={log} name="Metrics" />
          <hr />
          <XVIZPanel log={log} name="Camera" />
          <hr />
          <XVIZPanel
            log={log}
            name="Tables"
            componentProps={tableComponentProps}
          />
          <hr />
          <Form
            data={APP_SETTINGS}
            values={this.state.settings}
            onChange={this._onSettingsChange}
          />
          <StreamSettingsPanel log={log} />
        </div>
        <div id="log-panel">
          <div id="map-view">
            <LogViewer
              log={log}
              showMap={false}
              car={CAR}
              xvizStyles={XVIZ_STYLE}
              showTooltip={settings.showTooltip}
              viewMode={VIEW_MODE[settings.viewMode]}
              customLayers={customLayers}
            />
            <div id="hud">
              <MeterWidget
                log={log}
                streamName="/vehicle/acceleration"
                label="Acceleration"
                min={-10}
                max={10}
              />
              <hr />
              <MeterWidget
                log={log}
                streamName="/vehicle/velocity"
                label="Speed"
                getWarning={x => (x > 6 ? "FAST" : "")}
                min={0}
                max={20}
              />
            </div>
          </div>
        </div>
      </div>
    );
  }
}

render(
  <ThemeProvider theme={UI_THEME}>
    <CarlaViz />
  </ThemeProvider>,
  document.getElementById("app")
);
