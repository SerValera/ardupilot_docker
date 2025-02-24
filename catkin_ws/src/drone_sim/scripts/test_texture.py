#!/usr/bin/env python3

def update_sdf_file(file_path, new_name):
    try:
        with open(file_path, 'r', encoding='utf-8') as file:
            lines = file.readlines()

        # Меняем нужные строки
        lines[9] = lines[9].replace("two.dae", f"{new_name}.dae")
        lines[16] = lines[16].replace("two.dae", f"{new_name}.dae")

        with open(file_path, 'w', encoding='utf-8') as file:
            file.writelines(lines)
        
        print(f"Файл {file_path} обновлён: заменено на {new_name}.dae")
    except Exception as e:
        print(f"Ошибка: {e}")

if __name__ == "__main__":
    file = "/home/sim/ardupilot_docker/catkin_ws/src/drone_sim/models/label/model.sdf"
    labels = ['one']

    for l in labels:
        update_sdf_file(file, l)

