#!/usr/bin/env python

from svgwrite import Drawing
import rospy
from nav_msgs.msg import OccupancyGrid
from yaml import load

from cv2 import imencode, imwrite, resize, INTER_CUBIC
from numpy import zeros, uint8
from base64 import b64encode


class Map2Svg:

    def __init__(self):
        self.map = None
        self.drawing = None
        self.margin = 500
        self.output_size_scale = .2

        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.occ_map_cb)
        self.background_img = None

    def occ_map_cb(self, msg):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        length = len(msg.data)
    #self.min_line = []

    #creat an mat to load costmap
        costmap_mat = zeros((height, width), uint8)

        for i in range(0, height):
            for j in range(0, width):
                p = msg.data[i * width + j]
                if p < 0:
                    v = 128
                elif p == 0:
                    v = 255
                else:
                    v = 0
                costmap_mat[height-i-1, width-j-1] = v
                # costmap_mat[height-i-1, width-j-1] = 255 - int(
                #     float(msg.data[i * width + j]) / 100 * 255
                #     )

        costmap_mat = resize(
            costmap_mat, None, fx=self.output_size_scale,
            fy=self.output_size_scale,
            interpolation=INTER_CUBIC)
        self.background_img = b64encode(
            imencode('.png', costmap_mat)[1].tostring())
        print len(self.background_img)
        imwrite('test.png', costmap_mat)
        print 'done'

    def load_file(self, filename):
        with open(filename, 'r') as file:
            data = file.read()
            self.map = load(data)

    def _create_svg(self):

        svg_objs = []
        nodes = {}

        boundaries = [
            float('inf'), float('inf'),
            float('-inf'), float('-inf')
        ]
        if self.map:
            factory = Drawing('topological.svg')
            for n in self.map:
                node = n['node']
                name = node['name']
                x = node['pose']['position']['x'] * 100
                y = node['pose']['position']['y'] * 100

                nodes[name] = node

                boundaries[0] = min(x, boundaries[0])
                boundaries[1] = min(y, boundaries[1])
                boundaries[2] = max(x, boundaries[2])
                boundaries[3] = max(y, boundaries[3])

                svg_objs.append(
                    factory.polygon(points=[
                            (v['x'] * 100 + x, v['y'] * 100 + y)
                            for v in node['verts']
                        ],
                        fill='#FF0000',
                        opacity=0.3
                    )
                )
                svg_objs.append(
                    factory.circle(
                        center=(x, y), r=10, fill='blue'
                    )
                )
                svg_objs.append(
                    factory.text(name, x=[x], y=[y], font_size=100)
                )

            for from_node in nodes.values():
                edges = from_node['edges']
                for e in edges:
                    to_node = nodes[e['node']]

                    from_pos = (
                        from_node['pose']['position']['x'] * 100,
                        from_node['pose']['position']['y'] * 100)
                    to_pos = (
                        to_node['pose']['position']['x'] * 100,
                        to_node['pose']['position']['y'] * 100)
                    stroke_color = (
                        'black' if e['action'] == 'move_base' else 'red'
                    )
                    svg_objs.append(
                        factory.line(
                            start=from_pos, end=to_pos,
                            opacity=0.2,
                            stroke_width=10,
                            stroke=stroke_color)
                    )

            self.drawing = Drawing(
                'topological.svg',
                size=(
                    (boundaries[2] - boundaries[0]) * self.output_size_scale,
                    (boundaries[3] - boundaries[1]) * self.output_size_scale),
                viewBox='%f %f %f %f' % (
                    boundaries[0] - self.margin,
                    boundaries[1] - self.margin,
                    boundaries[2] - boundaries[0] + self.margin * 2,
                    boundaries[3] - boundaries[1] + self.margin * 2)
            )

            for o in svg_objs:
                self.drawing.add(o)
            #self.drawing.add(self.drawing.text(name, x=[x], y=[y]))

    def write_svg(self):
        if self.drawing:
            self.drawing.save()


if __name__ == "__main__":
    rospy.init_node('top2svg')
    m2s = Map2Svg()
    #m2s.load_file('test.yaml')
    #m2s._create_svg()
    #m2s.write_svg()
    rospy.spin()
# dwg = svgwrite.Drawing('test.svg')
# dwg.add(dwg.line((30, 30), (10, 30), stroke=svgwrite.rgb(10, 10, 16, '%')))
# dwg.add(dwg.text('Test', insert=(10, 0.2), fill='red'))
# dwg.save()
