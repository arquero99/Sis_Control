with DC_motor_sim, Controller, Ada.Text_IO;
use Ada.Text_IO;


package body Simulator_operator is

   procedure Carry_out_a_simulation(Kp, Ki, Kd: in Real; Score: out Real) is

      speed: array(1..2000) of Real; -- Samples during 0.2s
      abort_simulation : boolean := false;
      Tp : integer;
      Tr, Ts : integer;
      Mp_Value, Mp : Real;
      Controller_reference : constant Real := 80.0;
      Expected_Tr : constant integer := 140; -- /12 ms
      Expected_Tp : constant integer := 193; -- /14 ms
      Expected_Mp : constant Real := 0.07*Controller_reference; -- 7%
      Expected_Ts : constant integer := 291; --/10 ms
   begin
      -- Init motor simulator and speed controller
      Dc_motor_sim.init;
      Controller.Init(Kp, Ki, Kd);

      -- Set controller reference
      Controller.Set_reference(Controller_reference);

      -- Init speed
      for x in 1..2000 loop
         speed(x) := 0.0;
      end loop;

      -- Simulate

      Controller.Exec_controller_cycle;
      DC_motor_sim.Exec_cycle;

      for x in 1..2000 loop
         speed(x) := DC_motor_sim.Give_me_speed;

         -- Verify extrem values
         if (speed(x) > Controller_reference*5.0) or (speed(x) < 0.0) then
            abort_simulation := true;
            exit; -- Abort simulation
         end if;

         if ((x mod 10) = 0) then
            -- The controller runs every milisecond
            Controller.Exec_controller_cycle;
         end if;
         DC_motor_sim.Exec_cycle;
      end loop;

      -- Evaluate results
      if abort_simulation then
         Score := Real'Last/2.0;
      else
         -- Look for rise time
         Tr := 0;


         for x in 2..1999 loop
            if (speed(x) >= Controller_reference) then --Detectar cuando cruza la linea de la velocidad objetivo
               Tr := x;
               exit;
            else
               Tr := Integer'Last/4;
            end if;
         end loop;


         -- Set Tp and Mp
         if (Tr > 0) and (Tr < 2000) then
            for x in Tr..2000 loop            -- Se parte de Tr
               if speed(x) <= speed(x-1) then -- Detectar cuando decae/se estabiliza la velocidad
                  Tp := x-1;
                  Mp := speed(x-1) - Controller_reference;
                  exit;
               end if;
            end loop;
         end if;


         -- Cálculo de Ts
         if (Tp > 0) and (Tp < 2000) then
            for x in reverse Tp..2000 loop --Dado que el tiempo de establecimiento es a la derecha del tiempo de pico se corta en este
               if abs(Controller_reference - speed(x)) <= Controller_reference*0.02 then
                  Ts := x;
               end if;
            end loop;
         else
            Ts := integer'Last/4;
         end if;

         -- Cálculo de Mp
         --if(Tr > 0) and (Tr < 2000) then
           -- for x in Tr..2000 loop
           --    if (speed(x) <= speed(x-1)) and speed(x) > Controller_reference then
           --       Mp := speed(x-1) - Controller_reference;
           --       exit;
           --    end if;
           -- end loop;
         --end if;

         if(Mp <= Expected_Mp) then
            Mp_Value := 0.0;                       --Premio si está por debajo de ese 7%.
         else
            Mp_Value := Real'Last/4;               --Castigo si está por encima.
         end if;

         --Cálculo de Score
         Score := Real(abs(Tr - Expected_Tr)) + Real(abs(Ts - Expected_Ts)) + Real(abs(Tp - Expected_Tp)) + Real(abs(Mp_Value));
      end if;

   end Carry_out_a_simulation;

end Simulator_operator;
