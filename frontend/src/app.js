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

import githubIcon from "../public/github_icon.png";

import "./index.css";

const backendHostname =
  __BACKEND_HOST__ === "" ? document.location.hostname : __BACKEND_HOST__;
const backendPort = __BACKEND_PORT__ === "" ? 8081 : __BACKEND_PORT__;

const carlaLog = new XVIZLiveLoader({
  logGuid: "mock",
  bufferLength: 10,
  serverConfig: {
    defaultLogLength: 50,
    serverUrl: "ws://" + backendHostname + ":" + backendPort
  },
  worker: true,
  maxConcurrency: 10
});

const tableComponentProps = {
  table: {
    height: 200
  },
  treetable: {
    height: 200
  }
};

class CarlaViz extends PureComponent {
  state = {
    log: carlaLog,
    metadataReceived: false,
    settings: {
      viewMode: "PERSPECTIVE",
      showTooltip: true
    }
  };

  async _setMapLayer(map) {
    this.forceUpdate();
    this.setState({
      map: new GeoJsonLayer({
        coordinateSystem: COORDINATE_SYSTEM.METER_OFFSETS,
        coordinateOrigin: [0, 0, 0],
        id: "carla_map",
        data: map,
        stroked: true,
        filled: true,
        wireframe: false,
        extruded: true,
        getFillColor: this._decideFillColor,
        getLineColor: this._decideGeoJsonLineColor, // [255, 255, 255, 255],
        getLineWidth: this._decideGeoJsonLineWidth,
        getElevation: this._decideElevation,
        getRadius: 0.00001,
        opacity: 10
      }),
      metadataReceived: true
    });
  }

  componentDidMount() {
    const { log } = this.state;
    log
      .on("ready", () => {
        const metadata = log.getMetadata();
        log.socket.onclose = () => {
          this.setState({
            metadataReceived: false
          });
        };
        if (metadata.map) {
          this.setState(
            {
              map: null,
              metadataReceived: false
            },
            () => {
              this._setMapLayer(metadata.map);
            }
          );
        } else {
          this.setState({
            metadataReceived: true
          });
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

  _onStreamSettingChange = changedSettings => {
    const { log } = this.state;
    if (log && log.isOpen()) {
      log.socket.send(JSON.stringify(changedSettings));
    } else {
      console.log("socket is closed");
    }
  };

  _decideGeoJsonLineColor = feature => {
    if (feature.properties.type === "road") {
      return [255, 255, 255, 255];
    } else if (feature.properties.type === "stop_sign") {
      return [255, 0, 0, 255];
    }
    return [255, 255, 255, 255];
  };

  _decideGeoJsonLineWidth = feature => {
    if (feature.properties.type === "road") {
      return 0.1;
    } else if (feature.properties.type === "stop_sign") {
      return 0.05;
    }
    return 0.1;
  };

  _decideElevation = feature => {
    if (Object.hasOwn(feature.properties, "height")) {
      return feature.properties.height;
    }
    return 0.0;
  };

  _decideFillColor = feature => {
    if (feature.properties.type === "building") {
      return [105, 105, 105, 255];
    } else if (feature.properties.type === "sidewalk") {
      return [112, 128, 144, 255];
    } else if (feature.properties.type === "plant") {
      return [0, 255, 255, 255];
    }
    return [255, 255, 255, 255];
  };

  render() {
    const { log, map, metadataReceived, settings } = this.state;
    let customLayers = [];
    if (map) {
      customLayers = [map];
    } else {
      customLayers = [];
    }

    return (
      <div id="container">
        <div id="control-panel">
          <div id="github">
            <p>
              <a href="https://github.com/mjxu96/carlaviz" target="_blank">
                CarlaViz
              </a>
            </p>
            <a href="https://github.com/mjxu96/carlaviz" target="_blank">
              <img src={githubIcon}></img>
            </a>
          </div>
          {metadataReceived ? (
            <div>
              <hr id="github-hr" />
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
              <StreamSettingsPanel
                log={log}
                onSettingsChange={this._onStreamSettingChange}
              />
            </div>
          ) : (
            <div>
              <h4>Launch the backend and refresh</h4>
            </div>
          )}
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
            {metadataReceived ? (
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
            ) : (
              <div></div>
            )}
          </div>
        </div>
        <div id="author-info">
          <p>Author: Minjun Xu</p>
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
