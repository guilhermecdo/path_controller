import math

# Função para simular a leitura do sensor LIDAR em graus específicos
def read_lidar(sensor_position, robot_position, obstacles):
    angles = list(range(0, 360))  # Leituras em graus de 0 a 359
    lidar_readings = []

    for angle in angles:
        x = robot_position[0] + sensor_position * math.cos(math.radians(angle))
        y = robot_position[1] + sensor_position * math.sin(math.radians(angle))

        # Verifica se há obstáculo na posição (x, y)
        obstacle_detected = False
        for obstacle in obstacles:
            dist_to_obstacle = math.sqrt((x - obstacle[0]) ** 2 + (y - obstacle[1]) ** 2)
            if dist_to_obstacle < obstacle[2]:  # Distância do obstáculo (raio do obstáculo)
                obstacle_detected = True
                break

        lidar_readings.append((angle, obstacle_detected))

    return lidar_readings

# Função para a fase de contorno do Bug2
def contour_phase(robot_position, goal_position, sensor_range, obstacles):
    # Distância mínima para considerar que o robô chegou ao objetivo
    distance_threshold = 0.1

    while True:
        # Mova o robô em linha reta em direção ao objetivo até encontrar um obstáculo ou atingir o limite do ambiente
        while True:
            distance_to_goal = math.sqrt((goal_position[0] - robot_position[0]) ** 2 + (goal_position[1] - robot_position[1]) ** 2)

            if distance_to_goal < distance_threshold:
                return  # O robô alcançou o objetivo

            # Verifique se há um obstáculo à frente do robô usando o sensor LIDAR
            lidar_readings = read_lidar(sensor_range, robot_position, obstacles)
            obstacle_ahead = any(reading[1] for reading in lidar_readings)

            if obstacle_ahead:
                break  # O robô encontrou um obstáculo

            # Atualize a posição do robô para se mover em linha reta
            robot_position = (
                robot_position[0] + 0.1 * math.cos(math.atan2(goal_position[1] - robot_position[1], goal_position[0] - robot_position[0])),
                robot_position[1] + 0.1 * math.sin(math.atan2(goal_position[1] - robot_position[1], goal_position[0] - robot_position[0]))
            )

        # Fase de contorno: siga a fronteira do obstáculo até encontrar uma abertura para alcançar o objetivo
        while True:
            # Verifique se há uma abertura para alcançar a posição mais próxima da meta
            closest_point_to_goal = min(lidar_readings, key=lambda reading: math.sqrt((goal_position[0] - robot_position[0] - sensor_range * math.cos(math.radians(reading[0]))) ** 2 + (goal_position[1] - robot_position[1] - sensor_range * math.sin(math.radians(reading[0]))) ** 2))
            angle_to_goal = closest_point_to_goal[0]

            # Mova o robô em linha reta em direção à abertura
            while True:
                distance_to_goal = math.sqrt((goal_position[0] - robot_position[0]) ** 2 + (goal_position[1] - robot_position[1]) ** 2)

                if distance_to_goal < distance_threshold:
                    return  # O robô alcançou o objetivo

                robot_position = (
                    robot_position[0] + 0.1 * math.cos(math.radians(angle_to_goal)),
                    robot_position[1] + 0.1 * math.sin(math.radians(angle_to_goal))
                )

                # Verifique se o robô encontrou um novo obstáculo durante o movimento
                lidar_readings = read_lidar(sensor_range, robot_position, obstacles)
                obstacle_ahead = any(reading[1] for reading in lidar_readings)

                if obstacle_ahead:
                    break  # O robô encontrou um novo obstáculo

# Exemplo de uso:
robot_position = (0, 0)
goal_position = (5, 5)
sensor_range = 2.5
obstacles = [(3, 3, 1)]  # Cada obstáculo é uma tupla (coordenada_x, coordenada_y, raio)

contour_phase(robot_position, goal_position, sensor_range, obstacles)
