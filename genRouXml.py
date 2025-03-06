import xml.etree.ElementTree as ET
import pandas as pd
from xml.dom import minidom  # 导入 minidom 用于格式化 XML

'''
生成规则：
1. from="WM1" to="WM5"，用(section13×2)×0.8
2. from="WM1" to="WO1"，用section12×3–section13×2
3. from="WM1" to="WO2"，用(section13×2)×0.2
4. from="WI1" to="WM5"，用(section14×3–section13×2)×0.8
5. from="WI1" to="WO2"，用(section14×3–section13×2)×0.2
6. from="EM1" to="EM5"，用section1×3
7. from="EM1" to="EO2"，用(section6×4-section4×2)×0.6
8. from="EI1" to="EO1"，用section9×5–section7×3
9. from="EI1" to="EO2"，用(section6×4-section4×2)×0.2
10.from="EI1" to="EM5"，用(section9×5–section7×3)×4 - (section6×4 - section4×2)×0.2
11.from="EI2" to="EO2"，用(section6×4-section4×2)×0.2
12.from="EI3" to="EM5"，用section3×3-section4×2

'''


def calculate_flow(section_id, t_h, df):
    """
    根据 section_id 和 t-h 计算流量值。

    :param section_id: 区段 ID
    :param t_h: 时间段 (1-6)
    :param df: 包含流量数据的 DataFrame
    :return: 计算后的流量值（四舍五入至 10 的倍数，最小值为 50）
    """
    # 获取对应 section_id 和 t-h 的 volume_lane 值
    volume_lane = df[(df['section_id'] == section_id) & (df['t-h'] == t_h)]['volume_lane'].values[0]

    # 根据 section_id 和公式计算流量
    if section_id == 13:
        flow = (volume_lane * 2)
    elif section_id == 12:
        flow = (volume_lane * 3) - (df[(df['section_id'] == 13) & (df['t-h'] == t_h)]['volume_lane'].values[0] * 2)
    elif section_id == 14:
        flow = (volume_lane * 3) - (df[(df['section_id'] == 13) & (df['t-h'] == t_h)]['volume_lane'].values[0] * 2)
    elif section_id == 1:
        flow = volume_lane * 3
    elif section_id == 6:
        flow = (volume_lane * 4) - (df[(df['section_id'] == 4) & (df['t-h'] == t_h)]['volume_lane'].values[0] * 2)
    elif section_id == 9:
        flow = (volume_lane * 5) - (df[(df['section_id'] == 7) & (df['t-h'] == t_h)]['volume_lane'].values[0] * 3)
    elif section_id == 3:
        flow = (volume_lane * 3) - (df[(df['section_id'] == 4) & (df['t-h'] == t_h)]['volume_lane'].values[0] * 2)
    else:
        flow = 0  # 默认值，可根据需要调整

    # 四舍五入至 10 的倍数，最小值为 50
    flow = max(50, round(flow / 10) * 10)
    return flow


def generate_flows(root, from_node, to_node, flow_value, vehicle_ratios, vehicle_types, begin, end):
    """
    生成特定路径的流量定义，并根据车辆比例分配流量。

    :param root: XML 根节点
    :param from_node: 起始节点
    :param to_node: 目标节点
    :param flow_value: 总流量
    :param vehicle_ratios: 车辆比例列表，如 [0.4, 0.2, 0.4]
    :param vehicle_types: 车辆类型列表，如 ["HV", "CV", "CAV"]
    :param begin: 开始时间
    :param end: 结束时间
    """
    for ratio, vtype in zip(vehicle_ratios, vehicle_types):
        flow = ET.SubElement(root, "flow", {
            "id": f"{vtype.lower()}_{from_node}_{to_node}_{begin}_{end}",
            "color": "1,0,0" if vtype == "HV" else "0,1,0" if vtype == "CV" else "0,0,1",
            "departLane": "random",
            "departSpeed": "desired",
            "begin": str(begin),
            "end": str(end),
            "vehsPerHour": str(int(flow_value * ratio)),
            "type": vtype,
            "from": from_node,
            "to": to_node
        })


def generate_xml(vehicle_ratios, output_file):
    """
    生成 XML 文件。

    :param vehicle_ratios: 车辆比例列表，如 [0.4, 0.2, 0.4]
    :param output_file: 输出文件路径
    """
    # 读取 Excel 文件
    df = pd.read_excel("../../../RefData/vsd-MAGICdata/vsd-afterCollection.xlsx")

    # 创建根节点
    root = ET.Element("routes", {
        "xmlns:xsd": "http://www.w3.org/2001/XMLSchema",
        "xmlns:xsi": "http://www.w3.org/2001/XMLSchema-instance"
    })

    # 定义车辆类型
    ET.SubElement(root, "vType", {
        "id": "HV", "accel": "2", "decel": "4.5", "sigma": "0.5", "length": "5", "maxSpeed": "20",
        "carFollowModel": "IDM", "actionStepLenth": "1", "lcStrategic": "0", "lcKeepRight": "0",
        "lcCooperative": "0.5", "lcSpeedGain": "1"
    })
    ET.SubElement(root, "vType", {
        "id": "CV", "accel": "2", "decel": "4.5", "sigma": "0.5", "length": "5", "maxSpeed": "20",
        "carFollowModel": "IDM", "actionStepLenth": "1", "lcStrategic": "0.5", "lcKeepRight": "0",
        "lcCooperative": "1", "lcSpeedGain": "0.5"
    })
    ET.SubElement(root, "vType", {
        "id": "CAV", "accel": "2", "decel": "4.5", "sigma": "0.5", "length": "5", "maxSpeed": "20",
        "minGap": "2", "carFollowModel": "Krauss", "actionStepLenth": "1", "lcStrategic": "0.5",
        "lcKeepRight": "0", "lcCooperative": "1", "lcSpeedGain": "0.5"
    })

    # 定义车辆类型列表
    vehicle_types = ["HV", "CV", "CAV"]

    # 定义路径和对应的 section_id 计算规则
    paths = [
        {"from": "WM1", "to": "WM5", "section_id": 13, "factor": 0.8},
        {"from": "WM1", "to": "WO1", "section_id": 12, "factor": 1},
        {"from": "WM1", "to": "WO2", "section_id": 13, "factor": 0.2},
        {"from": "WI1", "to": "WM5", "section_id": 14, "factor": 0.8},
        {"from": "WI1", "to": "WO2", "section_id": 14, "factor": 0.2},
        {"from": "EM1", "to": "EM5", "section_id": 1, "factor": 1},
        {"from": "EM1", "to": "EO2", "section_id": 6, "factor": 0.6},
        {"from": "EI1", "to": "EO1", "section_id": 9, "factor": 1},
        {"from": "EI1", "to": "EO2", "section_id": 6, "factor": 0.2},
        {"from": "EI2", "to": "EO2", "section_id": 6, "factor": 0.2},
        {"from": "EI3", "to": "EM5", "section_id": 3, "factor": 1}
    ]

    # 遍历时间段 (1-6)
    for t_h in range(1, 7):
        begin = (t_h - 1) * 1800  # 开始时间
        end = t_h * 1800  # 结束时间

        # 遍历路径并生成流量
        for path in paths:
            flow_value = calculate_flow(path["section_id"], t_h, df) * path["factor"]
            generate_flows(root, path["from"], path["to"], flow_value, vehicle_ratios, vehicle_types, begin, end)

    # 使用 minidom 格式化 XML
    xml_str = ET.tostring(root, encoding='utf-8', method='xml')
    parsed_xml = minidom.parseString(xml_str)
    pretty_xml_str = parsed_xml.toprettyxml(indent="    ")  # 使用四个空格缩进

    # 写入格式化后的 XML 文件
    with open(output_file, "w", encoding="utf-8") as f:
        f.write(pretty_xml_str)


if __name__ == "__main__":
    vehicle_ratios = [0.6, 0.2, 0.2]  # HV, CV, CAV 的比例
    # 调用函数生成 XML 文件
    generate_xml(vehicle_ratios, "MainFile/MAGIC.rou.xml")