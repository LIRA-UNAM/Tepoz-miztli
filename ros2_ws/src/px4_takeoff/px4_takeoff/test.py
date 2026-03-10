def timer_cb(self):
        now = self.get_clock().now().nanoseconds // 1000

        # MODO OFFBOARD
        offboard = OffboardControlMode()
        offboard.timestamp = now
        offboard.position = True 
        offboard.velocity = True # ¡CAMBIO CRÍTICO! Debe ser True si envías velocidades en TAKEOFF/LAND
        offboard.acceleration = False
        self.offboard_pub.publish(offboard)

        # SETPOINT
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = now

        # Capturamos la posición real (X, Y, Yaw) justo antes de despegar
        if self.state == "INIT" or self.state == "ARMING":
            self.locked_x = self.current_x
            self.locked_y = self.current_y
            self.locked_yaw = self.current_yaw

        # Asignamos de forma segura las variables bloqueadas
        safe_x = self.locked_x if self.locked_x is not None else 0.0
        safe_y = self.locked_y if self.locked_y is not None else 0.0
        setpoint.yaw = self.locked_yaw if self.locked_yaw is not None else 0.0
        
        # ¡CAMBIO CRÍTICO! En lugar de NaN, obligamos al dron a "quedarse donde está"
        setpoint.position = [safe_x, safe_y, self.current_z]
        setpoint.velocity = [0.0, 0.0, 0.0] # Mandamos 0 en lugar de NaN
        setpoint.acceleration = [float('nan'), float('nan'), float('nan')]
        setpoint.jerk = [float('nan'), float('nan'), float('nan')]
        setpoint.yawspeed = float('nan')

        # MÁQUINA DE ESTADOS
        if self.state == "INIT":
            if self.counter > 20:
                self.send_cmd(176, param1=1.0, param2=6.0) # Entrar a Offboard
                self.state = "ARMING"

        elif self.state == "ARMING":
            if self.counter > 30:
                self.send_cmd(400, param1=1.0) # Armar motores
                self.get_logger().info("ARMED - Ascendiendo a 2.5m")
                self.state = "TAKEOFF"

        elif self.state == "TAKEOFF":
            # Subimos anclados a la posición X y Y real capturada en piso
            setpoint.position = [safe_x, safe_y, self.target_z]
            setpoint.velocity = [0.0, 0.0, -0.8] # Inyección de potencia para subir

            if abs(self.current_z - self.target_z) < 0.15: 
                self.state = "HOLD"
                self.get_logger().info("HOLD POSITION - Manteniendo altura y posición")

        elif self.state == "HOLD":
            # Mantenemos las coordenadas de origen real y la altura meta
            setpoint.position = [safe_x, safe_y, self.target_z]
            setpoint.velocity = [0.0, 0.0, 0.0] # Aseguramos que la velocidad sea 0
                        
            self.hold_counter += 1
            pass_time = self.hold_counter * 0.1 # A 10Hz, 1 tick es 0.1s
            if pass_time >= self.hold_duration:
                self.state = "LAND"
                self.get_logger().info("LANDING - Aterrizando suavemente")

        elif self.state == "LAND":
            # Aterrizamos bajando exactamente sobre las mismas coordenadas bloqueadas
            setpoint.position = [safe_x, safe_y, 0.0] # Ojo, 0.0 z asume que el origen está en el piso
            setpoint.velocity = [0.0, 0.0, 0.4] # Velocidad de descenso controlada
            
            if self.current_z > -0.20:
                self.state = "LANDED"
                self.send_cmd(400, param1=0.0) # Desarmar motores
                self.get_logger().info("LANDING COMPLETED - Motores desarmados")

        self.trajectory_pub.publish(setpoint)
        self.counter += 1