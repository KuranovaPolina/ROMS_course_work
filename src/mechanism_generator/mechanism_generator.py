from lxml import etree
import numpy as np

class MuJoCoMechanismGenerator:
    def __init__(self):
        # Создаем корневой элемент
        self.root = etree.Element("mujoco")
        
        # Добавляем базовые настройки
        self._add_basic_settings()
        
        # Создаем словарь для хранения элементов
        self.elements = {
            'bodies': [],
            'joints': [],
            'geoms': [],
            'sites': [],
            'tendons': []
        }

    def _add_basic_settings(self):
        # Добавляем опции
        option = etree.SubElement(self.root, "option")
        option.set("gravity", "0 0 -9.81")
        option.set("timestep", "0.002")
        flag = etree.SubElement(option, "flag")
        flag.set("warmstart", "enable")
        # Отключаем контакты между суставами (глобальная настройка)
        flag.set("contact", "disable")

        # Добавляем визуальные настройки
        visual = etree.SubElement(self.root, "visual")
        global_visual = etree.SubElement(visual, "global")
        global_visual.set("offwidth", "800")
        global_visual.set("offheight", "800")
        quality = etree.SubElement(visual, "quality")
        quality.set("shadowsize", "4096")
        map_visual = etree.SubElement(visual, "map")
        map_visual.set("force", "0.1")
        map_visual.set("fogstart", "3")
        map_visual.set("fogend", "5")
        map_visual.set("znear", "0.1")
        map_visual.set("zfar", "50")

        # Добавляем ассеты
        self._add_assets()

    def _add_assets(self):
        asset = etree.SubElement(self.root, "asset")
        
        # Добавляем текстуры
        skybox = etree.SubElement(asset, "texture")
        skybox.set("type", "skybox")
        skybox.set("builtin", "gradient")
        skybox.set("rgb1", "0.3 0.5 0.7")
        skybox.set("rgb2", "0 0 0")
        skybox.set("width", "512")
        skybox.set("height", "512")

        # Добавляем материалы
        materials = {
            'arm1': "0.8 0.3 0.3 1",
            'arm2': "0.3 0.8 0.3 1",
            'arm3': "0.3 0.3 0.8 1"
        }
        
        for name, rgba in materials.items():
            material = etree.SubElement(asset, "material")
            material.set("name", name)
            material.set("rgba", rgba)

    def add_body(self, name, pos, parent=None):
        """Добавляет тело в механизм"""
        if parent is None:
            parent = self.root.find(".//worldbody")
            if parent is None:
                parent = etree.SubElement(self.root, "worldbody")
        
        body = etree.SubElement(parent, "body")
        body.set("name", name)
        body.set("pos", " ".join(map(str, pos)))
        self.elements['bodies'].append(body)
        return body

    def add_joint(self, name, type, axis, pos, parent_body):
        """Добавляет сустав к телу"""
        joint = etree.SubElement(parent_body, "joint")
        joint.set("name", name)
        joint.set("type", type)
        joint.set("axis", " ".join(map(str, axis)))
        joint.set("pos", " ".join(map(str, pos)))
        self.elements['joints'].append(joint)
        return joint

    def add_geom(self, type, size, pos=None, fromto=None, material=None, parent_body=None):
        """Добавляет геометрию к телу"""
        if parent_body is None:
            parent_body = self.elements['bodies'][-1]
            
        geom = etree.SubElement(parent_body, "geom")
        geom.set("type", type)
        
        if isinstance(size, (list, tuple)):
            geom.set("size", " ".join(map(str, size)))
        else:
            geom.set("size", str(size))
            
        if pos is not None:
            geom.set("pos", " ".join(map(str, pos)))
        if fromto is not None:
            geom.set("fromto", " ".join(map(str, fromto)))
        if material is not None:
            geom.set("material", material)
            
        self.elements['geoms'].append(geom)
        return geom

    def add_site(self, name, pos, parent_body=None):
        """Добавляет сайт к телу"""
        if parent_body is None:
            parent_body = self.elements['bodies'][-1]
            
        site = etree.SubElement(parent_body, "site")
        site.set("name", name)
        site.set("pos", " ".join(map(str, pos)))
        self.elements['sites'].append(site)
        return site

    def add_tendon(self, name, site1, site2, limited=True, range_vals=(0, 0.0000001)):
        """Добавляет сухожилие между двумя сайтами"""
        tendon = etree.SubElement(self.root, "tendon")
        spatial = etree.SubElement(tendon, "spatial")
        spatial.set("name", name)
        spatial.set("limited", str(limited).lower())
        spatial.set("range", " ".join(map(str, range_vals)))
        spatial.set("width", "0.003")
        spatial.set("rgba", "0.8 0.8 0.8 1")
        
        site1_elem = etree.SubElement(spatial, "site")
        site1_elem.set("site", site1)
        site2_elem = etree.SubElement(spatial, "site")
        site2_elem.set("site", site2)
        
        self.elements['tendons'].append(spatial)
        return spatial

    def add_motor(self, joint_name, name=None, gear=1, ctrlrange=(-0.001, 0.001)):
        """Добавляет мотор для сустава"""
        actuator = self.root.find(".//actuator")
        if actuator is None:
            actuator = etree.SubElement(self.root, "actuator")
            
        motor = etree.SubElement(actuator, "motor")
        motor.set("joint", joint_name)
        if name is None:
            name = f"motor_{joint_name}"
        motor.set("name", name)
        motor.set("gear", str(gear))
        motor.set("ctrlrange", " ".join(map(str, ctrlrange)))
        return motor

    def save_xml(self, filename):
        """Сохраняет XML файл"""
        tree = etree.ElementTree(self.root)
        tree.write(filename, pretty_print=True, xml_declaration=True, encoding='utf-8')

# Пример использования:
if __name__ == "__main__":
    # Создаем генератор механизма
    generator = MuJoCoMechanismGenerator()
    
    # Добавляем базовое тело
    base = generator.add_body("base", [0, 0, 0.1])
    generator.add_geom("cylinder", [0.05, 0.01], material="arm1", parent_body=base)
    joint1 = generator.add_joint("joint1", "hinge", [0, 0, 1], [0, 0, -0.05], base)
    generator.add_geom("capsule", 0.01, fromto=[0, 0, 0, 0, 0.25, 0], material="arm2", parent_body=base)
    
    # Добавляем первое звено
    link1 = generator.add_body("link1", [0, 0.25, 0], parent=base)
    generator.add_geom("cylinder", [0.05, 0.01], material="arm1", parent_body=link1)
    joint2 = generator.add_joint("joint2", "hinge", [0, 0, 1], [0, 0, -0.05], link1)
    generator.add_geom("capsule", 0.01, fromto=[0, 0, 0, 0.25, 0, 0], material="arm2", parent_body=link1)
    
    # Добавляем второе звено
    link2 = generator.add_body("link2", [0.25, 0, 0], parent=link1)
    generator.add_geom("cylinder", [0.05, 0.01], material="arm1", parent_body=link2)
    joint3 = generator.add_joint("joint3", "hinge", [0, 0, 1], [0, 0, -0.05], link2)
    generator.add_geom("capsule", 0.01, fromto=[0, 0, 0, 0, -0.25, 0], material="arm2", parent_body=link2)
    
    # Добавляем сайты
    site1 = generator.add_site("c1", [0, -0.25, 0], link2)
    
    # Добавляем конечное звено
    end = generator.add_body("end", [0.25, 0, 0.1])
    generator.add_geom("cylinder", [0.05, 0.01], material="arm1", parent_body=end)
    joint4 = generator.add_joint("joint4", "hinge", [0, 0, 1], [0, 0, -0.05], end)
    site2 = generator.add_site("c2", [0, 0, 0], end)
    
    # Добавляем сухожилие
    generator.add_tendon("connection", "c1", "c2")
    
    # Добавляем моторы с пониженной скоростью
    for joint_id, data in self.joints.items():
        if data['motor']:
            self.generator.add_motor(
                f"joint{joint_id}", 
                name=f"motor_joint{joint_id}", 
                gear=10,                    # Уменьшенное передаточное число
                ctrlrange=(-0.001, 0.001)   # Суженный диапазон управления
            )
    
    # Сохраняем XML файл
    generator.save_xml("generated_mechanism.xml") 