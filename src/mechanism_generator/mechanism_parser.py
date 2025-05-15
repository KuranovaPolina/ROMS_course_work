from lxml import etree
import numpy as np
from mechanism_generator import MuJoCoMechanismGenerator

class MechanismParser:
    def __init__(self):
        self.joints = {}  # Словарь для хранения информации о суставах
        self.generator = MuJoCoMechanismGenerator()
        
    def parse_joint(self, line):
        """Парсит строку с информацией о суставе"""
        # Удаляем комментарии и лишние пробелы
        line = line.split('#')[0].strip()
        if not line:  # Пропускаем пустые строки
            return
            
        parts = line.split()
        if len(parts) < 6:
            raise ValueError(f"Неверный формат строки: {line}. Ожидается минимум 6 параметров")
            
        try:
            joint_id = int(parts[0])
            pos_x = float(parts[1])
            pos_y = float(parts[2])
            motor = int(parts[3])
            is_base = int(parts[4])
            is_end = int(parts[5])
            connections = [int(x) for x in parts[6:]] if len(parts) > 6 else []
            
            # Проверка значений
            if motor not in [0, 1]:
                raise ValueError(f"Значение motor должно быть 0 или 1, получено: {motor}")
            if is_base not in [0, 1]:
                raise ValueError(f"Значение base должно быть 0 или 1, получено: {is_base}")
            if is_end not in [0, 1]:
                raise ValueError(f"Значение end должно быть 0 или 1, получено: {is_end}")
            
            self.joints[joint_id] = {
                'pos': [pos_x, pos_y, 0.1],  # Добавляем z-координату
                'motor': bool(motor),
                'is_base': bool(is_base),
                'is_end': bool(is_end),
                'connections': connections
            }
        except ValueError as e:
            raise ValueError(f"Ошибка в строке '{line}': {str(e)}")
        
    def parse_file(self, filename):
        """Парсит файл с описанием механизма"""
        try:
            with open(filename, 'r') as f:
                for line_num, line in enumerate(f, 1):
                    try:
                        self.parse_joint(line)
                    except ValueError as e:
                        print(f"Ошибка в строке {line_num}: {str(e)}")
                        raise
        except FileNotFoundError:
            raise FileNotFoundError(f"Файл {filename} не найден")
                    
    def build_mechanism(self):
        """Строит механизм на основе распарсенных данных"""
        if not self.joints:
            raise ValueError("Нет данных о суставах для построения механизма")
        
        # Подготавливаем структуры данных
        self.bodies = {}
        self.sites = {}
        self.end_sites = {}
        self.visited = set()  # Для отслеживания уже созданных тел
        self.capsule_connections = set()  # Для отслеживания созданных капсул
        
        # Добавляем настройки физики для стабильности
        option = etree.SubElement(self.generator.root, "option")
        option.set("gravity", "0 0 -9.81")
        option.set("timestep", "0.001")
        flag = etree.SubElement(option, "flag")
        flag.set("warmstart", "enable")
        
        # Добавляем демпфирование для суставов
        default = etree.SubElement(self.generator.root, "default")
        joint = etree.SubElement(default, "joint")
        joint.set("damping", "1.0")
        joint.set("armature", "0.1")
        joint.set("frictionloss", "0.1")
        
        # Получаем worldbody
        worldbody = self.generator.root.find(".//worldbody")
        if worldbody is None:
            worldbody = etree.SubElement(self.generator.root, "worldbody")
            
        # 1. Создание тел
        # 1.1 Проходим по порядку id
        # 1.2 Ищем base joint
        try:
            base_id = next(id for id, data in self.joints.items() if data['is_base'])
        except StopIteration:
            raise ValueError("Не найден базовый сустав (is_base=1)")
            
        # 1.3 Устанавливаем base в корне worldbody
        base_data = self.joints[base_id]
        base_body = etree.SubElement(worldbody, "body")
        base_body.set("name", "base")
        base_body.set("pos", f"{base_data['pos'][0]} {base_data['pos'][1]} {base_data['pos'][2]}")
        
        # Добавляем геометрию для базового сустава
        base_geom = etree.SubElement(base_body, "geom")
        base_geom.set("name", "base_geom")
        base_geom.set("type", "cylinder")
        base_geom.set("size", "0.05 0.01")
        base_geom.set("material", "arm1")
        
        # Добавляем шарнир
        base_joint = etree.SubElement(base_body, "joint")
        base_joint.set("name", f"joint{base_id}")
        base_joint.set("type", "hinge")
        base_joint.set("axis", "0 0 1")
        base_joint.set("pos", "0 0 -0.05")
        
        # Добавляем сайт для соединений
        base_site = etree.SubElement(base_body, "site")
        base_site.set("name", f"site_{base_id}")
        base_site.set("pos", "0 0 0")
        self.sites[base_id] = f"site_{base_id}"
        
        # Сохраняем тело и отмечаем как посещенное
        self.bodies[base_id] = base_body
        self.visited.add(base_id)
        
        print(f"Создано базовое тело (ID: {base_id})")
        
        # 1.4 Добавляем конечные точки в worldbody
        end_joint_ids = [id for id, data in self.joints.items() if data['is_end']]
        for end_id in end_joint_ids:
            data = self.joints[end_id]
            
            # Создаем тело для конечной точки
            end_body = etree.SubElement(worldbody, "body")
            end_body.set("name", f"end{end_id}")
            end_body.set("pos", f"{data['pos'][0]} {data['pos'][1]} {data['pos'][2]}")
            
            # Создаем геометрию для конечной точки
            end_geom = etree.SubElement(end_body, "geom")
            end_geom.set("name", f"end{end_id}_geom")
            end_geom.set("type", "cylinder")
            end_geom.set("size", "0.05 0.01")
            end_geom.set("material", "arm1")
            
            # Добавляем сайт для соединения
            end_site = etree.SubElement(end_body, "site")
            end_site.set("name", f"site_{end_id}")
            end_site.set("pos", "0 0 0")
            
            # Сохраняем информацию о сайте и теле
            self.end_sites[end_id] = f"site_{end_id}"
            self.bodies[end_id] = end_body
            self.visited.add(end_id)
            
            print(f"Создана конечная точка (ID: {end_id})")
        
        # 1.5-1.8 Рекурсивно создаем тела и капсулы, начиная с базового
        self._build_link_hierarchy(base_id, base_body)
        
        # 2. Создаем тендоны к конечным точкам
        self._create_tendons(end_joint_ids)
        
        # 3. Добавляем моторы с управлением по позиции
        for joint_id, data in self.joints.items():
            if data['motor']:
                # Создаем актуатор для управления по позиции
                velocity_actuator = etree.SubElement(self._get_or_create_actuator(), "position")
                velocity_actuator.set("name", f"vel_joint{joint_id}")
                velocity_actuator.set("joint", f"joint{joint_id}")
                velocity_actuator.set("kv", "10.0")  # Коэффициент дифференциального усиления
                velocity_actuator.set("ctrlrange", "-3.14 3.14")
                
                print(f"Добавлен актуатор позиции для сустава {joint_id}")
        
        print("\nСтруктура созданного механизма:")
        print(f"- Тела: {list(self.bodies.keys())}")
        print(f"- Конечные точки: {list(self.end_sites.keys())}")
        
        return self.generator
    
    def _build_link_hierarchy(self, parent_id, parent_body):
        """
        Рекурсивно создает тела и капсулы по иерархии родитель-ребенок
        
        :param parent_id: ID родительского сустава
        :param parent_body: XML элемент родительского тела
        """
        parent_data = self.joints[parent_id]
        parent_pos = parent_data['pos']
        
        # Проходим по всем соединениям родительского сустава
        for child_id in parent_data['connections']:
            # Пропускаем уже созданные тела
            if child_id in self.visited:
                # Если ребенок уже создан, но это конечная точка, добавляем капсулу с сайтом
                if self.joints[child_id]['is_end']:
                    child_pos = self.joints[child_id]['pos']
                    
                    # Создаем вектор от родителя к ребенку
                    vec = [
                        child_pos[0] - parent_pos[0],
                        child_pos[1] - parent_pos[1],
                        child_pos[2] - parent_pos[2]
                    ]
                    
                    # Добавляем капсулу с сайтом в родительское тело
                    self._add_capsule_with_site(parent_body, parent_id, child_id, vec)
                    self.capsule_connections.add((parent_id, child_id))
                    
                    print(f"Добавлена капсула от {parent_id} к конечной точке {child_id}")
                    
                continue
            
            child_data = self.joints[child_id]
            
            # Если ребенок - конечная точка, просто добавляем капсулу с сайтом
            if child_data['is_end']:
                child_pos = child_data['pos']
                
                # Создаем вектор от родителя к ребенку
                vec = [
                    child_pos[0] - parent_pos[0],
                    child_pos[1] - parent_pos[1],
                    child_pos[2] - parent_pos[2]
                ]
                
                # Добавляем капсулу с сайтом в родительское тело
                self._add_capsule_with_site(parent_body, parent_id, child_id, vec)
                self.capsule_connections.add((parent_id, child_id))
                
                print(f"Добавлена капсула от {parent_id} к конечной точке {child_id}")
                
                continue
            
            # Вычисляем относительное положение ребенка относительно родителя
            rel_pos = [
                child_data['pos'][0] - parent_pos[0],
                child_data['pos'][1] - parent_pos[1],
                0
            ]
            
            # Создаем дочернее тело
            child_body = etree.SubElement(parent_body, "body")
            child_body.set("name", f"link{child_id}")
            child_body.set("pos", f"{rel_pos[0]} {rel_pos[1]} {rel_pos[2]}")
            
            # Добавляем геометрию
            child_geom = etree.SubElement(child_body, "geom")
            child_geom.set("name", f"link{child_id}_geom")
            child_geom.set("type", "cylinder")
            child_geom.set("size", "0.05 0.01")
            child_geom.set("material", "arm1")
            
            # Добавляем сустав
            child_joint = etree.SubElement(child_body, "joint")
            child_joint.set("name", f"joint{child_id}")
            child_joint.set("type", "hinge")
            child_joint.set("axis", "0 0 1")
            child_joint.set("pos", "0 0 -0.05")
            
            # Добавляем сайт для соединений
            child_site = etree.SubElement(child_body, "site")
            child_site.set("name", f"site_{child_id}")
            child_site.set("pos", "0 0 0")
            self.sites[child_id] = f"site_{child_id}"
            
            # Добавляем капсулу от родителя к ребенку
            self._add_capsule(parent_body, parent_id, child_id, rel_pos)
            self.capsule_connections.add((parent_id, child_id))
            
            # Сохраняем тело и отмечаем как посещенное
            self.bodies[child_id] = child_body
            self.visited.add(child_id)
            
            print(f"Создано тело {child_id} внутри тела {parent_id}")
            
            # Рекурсивно создаем тела для дочерних суставов
            self._build_link_hierarchy(child_id, child_body)
    
    def _add_capsule(self, body, from_id, to_id, vec):
        """
        Добавляет капсулу от текущего тела к дочернему
        
        :param body: XML элемент тела
        :param from_id: ID исходного сустава
        :param to_id: ID целевого сустава
        :param vec: Вектор от исходного к целевому суставу [x, y, z]
        """
        # Создаем капсулу внутри тела
        capsule = etree.SubElement(body, "geom")
        capsule.set("name", f"capsule_{from_id}_{to_id}")
        capsule.set("type", "capsule")
        capsule.set("fromto", f"0 0 0 {vec[0]} {vec[1]} {vec[2]}")
        capsule.set("size", "0.01")
        capsule.set("material", "arm2")
        
        print(f"Создана капсула от {from_id} к {to_id}")
    
    def _add_capsule_with_site(self, body, from_id, to_id, vec):
        """
        Добавляет капсулу с сайтом на конце для соединения с конечной точкой
        
        :param body: XML элемент тела
        :param from_id: ID исходного сустава
        :param to_id: ID целевого сустава (конечная точка)
        :param vec: Вектор от исходного к целевому суставу [x, y, z]
        """
        # Создаем капсулу внутри тела
        capsule = etree.SubElement(body, "geom")
        capsule.set("name", f"capsule_{from_id}_{to_id}")
        capsule.set("type", "capsule")
        capsule.set("fromto", f"0 0 0 {vec[0]} {vec[1]} {vec[2]}")
        capsule.set("size", "0.01")
        capsule.set("material", "arm2")
        
        # Создаем сайт в конце капсулы для соединения с конечной точкой
        conn_site = etree.SubElement(body, "site")
        conn_site.set("name", f"conn_site_{from_id}_{to_id}")
        conn_site.set("pos", f"{vec[0]} {vec[1]} {vec[2]}")
        
        print(f"Создан сайт соединения {from_id} -> {to_id} в позиции {vec[0]} {vec[1]} {vec[2]}")
    
    def _create_tendons(self, end_joint_ids):
        """
        Создает тендоны между сайтами в обычных телах и конечными точками
        
        :param end_joint_ids: Список ID конечных точек
        """
        tendon_section = self._get_or_create_tendon()
        
        # Проходим по всем соединениям с конечными точками
        for source_id, target_id in self.capsule_connections:
            # Проверяем, что целевой сустав - конечная точка
            if target_id in end_joint_ids:
                site_name = f"conn_site_{source_id}_{target_id}"
                
                # Создаем тендон между сайтом в обычном теле и конечной точкой
                tendon = etree.SubElement(tendon_section, "spatial")
                tendon.set("name", f"tendon_{source_id}_{target_id}")
                tendon.set("limited", "true")
                tendon.set("range", "0 0.0001")
                tendon.set("width", "0.003")
                tendon.set("rgba", "0.8 0.8 0.8 1")
                
                # Сайт в обычном теле (конец капсулы)
                site1 = etree.SubElement(tendon, "site")
                site1.set("site", site_name)
                
                # Сайт в конечной точке
                site2 = etree.SubElement(tendon, "site")
                site2.set("site", self.end_sites[target_id])
                
                print(f"Создан тендон от {source_id} к конечной точке {target_id}")
    
    def _get_or_create_actuator(self):
        """Возвращает или создает раздел actuator"""
        actuator = self.generator.root.find(".//actuator")
        if actuator is None:
            actuator = etree.SubElement(self.generator.root, "actuator")
        return actuator
    
    def _get_or_create_tendon(self):
        """Возвращает или создает раздел tendon"""
        tendon = self.generator.root.find(".//tendon")
        if tendon is None:
            tendon = etree.SubElement(self.generator.root, "tendon")
        return tendon
    
    def save_mechanism(self, filename):
        """Сохраняет механизм в XML файл"""
        self.generator.save_xml(filename)
        print(f"Механизм сохранен в файл {filename}")

def create_example_mechanism():
    """Создает пример механизма"""
    parser = MechanismParser()
    
    # Пример описания механизма
#     mechanism_description = """# joint_id pos_x pos_y motor base end connections
# 1 0.0 0.0 1 1 0 3     # Базовое звено с мотором (кривошип)
# 2 0.0 1.0 0 0 0 3 4  # Шатун (соединительное звено)
# 3 1.0 1.0 0 0 0 1 2   # Коромысло (качающееся звено)
# 4 1.0 0.0 0 0 1 2"""

    mechanism_description = """# joint_id pos_x pos_y motor base end connections
1 -0.3950124685830086 -0.3092843358865642 1 1 0 2 3
2 -1.0103167186060211 0.36613440005517417 0 0 1 1 4
3 0.21329809092020777 0.37243998680876184 1 0 0 1 4
4 2.0 -1.7062144791158373 0 0 0 2 3"""
    
    # Сохраняем описание во временный файл
    with open("mechanism_description.txt", "w") as f:
        f.write(mechanism_description)
    
    # Парсим и создаем механизм
    parser.parse_file("mechanism_description.txt")
    parser.build_mechanism()
    parser.save_mechanism("parsed_mechanism.xml")

if __name__ == "__main__":
    create_example_mechanism()