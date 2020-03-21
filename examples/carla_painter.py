'''
File: carla_painter.py
Author: Minjun Xu (mjxu96@gmail.com)
File Created: Monday, 7th October 2019 7:17:32 pm
'''

from websocket import create_connection
import logging
import json

class CarlaPainter(object):
    def __init__(self, host, port):
        self._set_up_logger()

        self._logger.info("Begin to connect to drawing server at %s:%d" % (host, port))
        self._ws = create_connection('ws://' + host + ':' + str(port))
        self._logger.info("Connected to drawing server at %s:%d" % (host, port))

    def draw_polylines(self, lines, color='#00FF00', width=2.5):
        """Draw several polylines in the CarlaViz frontend

        Args:
            lines (list): list of polylines to draw. Should be in the format of
                          [line_1, line_2, line_3...]. Every line should be in the
                          format of [point_1, point_2, point_3...]. Every point should
                          be in the format of [p_x, p_y, p_z].
            color (str, optional): color represented in str. Defaults to '#00FF00'.
            width (float, optional): line width. Defaults to 2.5.

        Raises:
            Exception: ValueError raised when the input format is not correct.
        """
        if not isinstance(lines, list):
            raise ValueError('input lines should be a list')
        if len(lines) == 0:
            self._logger.warning('no lines will be drawn')
            return
        if isinstance(lines[0], list) and not isinstance(lines[0][0], list):
            data_dict = {}
            data_dict['type'] = 'line'
            data_dict['vertices'] = [lines]
            data_dict['color'] = color
            data_dict['width'] = width
            try:
                self._ws.send(json.dumps(data_dict))
            except Exception as e:
                self._logger.warning(e)
        else:
            self._draw_polylines(lines, color, width)

    def draw_points(self, points):
        """Draw several points in the CarlaViz frontend

        Args:
            points (list): list of point to draw. Should be in the format of
                           [point_1, point_2, point_3...]. Every point should
                           be in the format of [p_x, p_y, p_z].

        Raises:
            Exception: ValueError raised when the input format is not correct.
        """
        if not isinstance(points, list):
            raise ValueError('input points should be a list')
        if len(points) == 0:
            self._logger.warning('no points will be drawn')
            return
        if isinstance(points[0], list) and not isinstance(points[0][0], list):
            self._draw_points([points])
        else:
            self._draw_points(points)

    def draw_texts(self, messages, positions, color='#fff', size=13):
        """Draw several texts in the CarlaViz frontend

        Args:
            messages (list): list of texts to draw. Should be in the format of
                             [str1, str2, str3...].
            positions (list): list of positions to draw. Should be in the format
                              of [point_1, point_2, point_3...]. Each point represents
                              the position of each text.
            color (str, optional): color of the texts. Defaults to '#fff'.
            size (int, optional): text size. Defaults to 13.
        """
        if isinstance(messages, str) and isinstance(positions, list):
            self._draw_texts([messages], [positions], color, size)
        else:
            self._draw_texts(messages, positions, color, size)

    def _draw_texts(self, messages, positions, color='#fff', size=13):
        if not isinstance(messages, list) or not isinstance(positions, list):
            raise ValueError('input messages and positions should be a list')
        if len(messages) != len(positions) or len(positions) == 0:
            raise ValueError('length of input messages and positions should be the same, length should not be 0')
        if not isinstance(positions[0], list) or len(positions[0]) != 3:
            raise ValueError('every position should be list of 3 numbers [x, y ,z]')
        data_dict = {}
        data_dict['type'] = 'text'
        data_dict['color'] = color
        data_dict['size'] = size
        data_dict['text'] = []
        for idx, message in enumerate(messages):
            data_dict['text'].append({
                'message': message,
                'position': positions[idx],
            })
        try:
            self._ws.send(json.dumps(data_dict))
        except Exception as e:
            self._logger.error(e)


    def _draw_points(self, points):
        data_dict = {}
        data_dict['type'] = 'point'
        data_dict['points'] = points
        try:
            self._ws.send(json.dumps(data_dict))
        except Exception as e:
            self._logger.error(e)


    def _draw_polylines(self, vertices, color='#00FF00', width=2.5):
        data_dict = {}
        data_dict['type'] = 'line'
        data_dict['vertices'] = vertices
        data_dict['color'] = color
        data_dict['width'] = width
        try:
            self._ws.send(json.dumps(data_dict))
        except Exception as e:
            self._logger.error(e)


    def _set_up_logger(self, log_level=logging.INFO):
        stream_handler = logging.StreamHandler()
        log_formatter = logging.Formatter(
            '[%(levelname)s] [PAINTER] [%(asctime)s] : %(message)s')
        stream_handler.setFormatter(log_formatter)
        stream_handler.setLevel(log_level)
        self._logger = logging.getLogger(__name__)
        self._logger.setLevel(log_level)
        self._logger.propagate = 0
        if not self._logger.handlers:
            self._logger.addHandler(stream_handler)
