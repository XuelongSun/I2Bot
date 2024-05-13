from controller import Supervisor

import numpy as np

from pompy import models, processors


class OdorPlume(Supervisor):
    def __init__(self, sim_region=(-6., 6., -2., 2.), max_num_puffs=200, puff_init_rad=0.02, odor_source=(-5.98, 0., 0.)):
        super(OdorPlume, self).__init__()
        self.timestep = int(self.getBasicTimeStep())
        self.puff_filed = self.getFromDef("Puffs").getField("children")
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(self.timestep)
        self.key_interval = 0.2
        self.last_key_time = 0
        self.loop_count = 0
        self.emit_odor = self.getDevice("odor_field")
        self.emit_wind = self.getDevice("wind_field")
        
        # set arena size
        self.arena = self.getFromDef("Arena")
        floor_size = [sim_region[1]-sim_region[0], sim_region[3]-sim_region[2]]
        self.arena.getField("floorSize").setSFVec2f(floor_size)
        
        # position of odor source
        self.getFromDef("odor_source").getField("translation").setSFVec3f([odor_source[0], odor_source[1], 0.06])
        
        # odor plume
        self.odor_source = odor_source
        self.odor_dt = self.timestep/1000.
        self.puff_update_interval = 2
        self.odor_rng = np.random.RandomState(20240111)
        self.wind_region = models.Rectangle(*sim_region)
        
        self.wind_model = models.WindModel(self.wind_region, int(floor_size[0]), int(floor_size[1]),
                                           u_av=2, v_av=0.,
                                           rng=self.odor_rng)
        self.odor_conc = processors.ConcentrationValueCalculator(1.)
        
        # let simulation run for 10s to equilibrate wind model
        for t in np.arange(0, 10, self.odor_dt):
            self.wind_model.update(self.odor_dt)
            # set up plume model
        self.plume_model = models.PlumeModel(self.wind_region, self.odor_source, self.wind_model, rng=self.odor_rng,
                                             max_num_puffs=max_num_puffs, puff_init_rad=puff_init_rad, 
                                             puff_release_rate=60, model_z_disp=False)
        # add puff to the webots scene
        for i in range(max_num_puffs):
            self.add_puff(puff_init_rad, self.odor_source[0], self.odor_source[1])
        
        # sample nodes
        self.sample_nodes = []
        for n, c in zip(['OSensorL', 'OSensorR'], [11, 12]):
            self.sample_nodes.append({
                'node': self.getFromDef(n),
                'channel': c
            })
        
    def update_puff(self):
        for i, p in enumerate(self.plume_model.puff_array):
            puff_node_ = self.puff_filed.getMFNode(i)
            puff_node_.getField("translation").setSFVec3f([p[0], p[1], p[3]**0.5])
            puff_node_.getField("size").setSFFloat(p[3]**0.5)

    def add_puff(self,size, x, y):
        puff_ = "puff {{size {} translation {} {} {}}}".format(size,x,y,size)
        self.puff_filed.importMFNodeFromString(-1, puff_)

    def clear_puff_nodes(self):
        for i, p in enumerate(self.plume_model.puff_array):
            puff_node_ = self.puff_filed.getMFNode(i)
            puff_node_.remove()
            
    def my_step(self):
        if self.step(self.timestep) == -1:
            return -1
        self.wind_model.update(self.odor_dt)
        self.plume_model.update(self.odor_dt)
        if self.loop_count % 20 == 0:
            self.send_odor_info()
        if self.loop_count % self.puff_update_interval == 0:
            self.update_puff()
        self.loop_count += 1
        return 0
    
    def send_odor_info(self):
        for v in self.sample_nodes:
            x, y, z = v['node'].getPosition()
            c_ = self.odor_conc.calc_conc_point(self.plume_model.puff_array, x, y)
            w_ = self.wind_model.velocity_at_pos(x, y)
            self.emit_odor.setChannel(v['channel'])
            self.emit_odor.send(f"{c_};{w_}")
            # print(f"send c{v['channel']} at ({x,y,z}):", c_, w_)

    def keyboard_control(self):
        key = self.keyboard.getKey()
        current_time = self.getTime()
        key_time = current_time - self.last_key_time > self.key_interval
        if key == ord('A') and key_time:
            size = np.random.uniform(0.05, 0.1)
            x = np.random.uniform(-4, 4)
            y = np.random.uniform(-2, 2)
            puff_ = "puff {{size {} translation {} {} {}}}".format(size,size,y,x)
            self.puff_filed.importMFNodeFromString(-1, puff_)
            self.last_key_time = current_time
        elif key == ord('S') and key_time:
            for v in self.sample_nodes:
                x, y, z = v['node'].getPosition()
                c_ = self.odor_conc.calc_conc_point(self.plume_model.puff_array, x, y)
                w_ = self.wind_model.velocity_at_pos(x, y)
                self.emit_odor.setChannel(v['channel'])
                self.emit_odor.send(f"{c_};{w_}")
                print(f"send c{v['channel']} at ({x,y,z}):", c_, w_)
            # x = self.agent.getField("translation").getSFVec3f()[0]
            # y = self.agent.getField("translation").getSFVec3f()[1]
            # c_ = self.odor_conc.calc_conc_point(self.plume_model.puff_array, x, y)
            # self.emit_odor.send(f"{c_}")
            # w_ = self.wind_model.velocity_at_pos(x, y)
            # self.emit_wind.send(f"{w_}")
            # print(f"Odor concentration: {c_}", f"Wind velocity: {w_}")
            self.last_key_time = current_time
        elif key == ord('C') and key_time:
            self.clear_puff_nodes()
            self.last_key_time = current_time

        elif key == ord("Q"):
            self.simulationQuit(0)
        elif key == ord("R") and key_time:
            self.simulationReset()
            self.last_key_time = current_time
        elif key == ord("P") and key_time:
            if self.isPaused():
                self.simulationSetMode(self.SIMULATION_MODE_REAL_TIME)
            else:
                self.simulationSetMode(self.SIMULATION_MODE_PAUSE)
            self.last_key_time = current_time


if __name__ == "__main__":
    sim_region=(-4, 4, -2, 2)
    odor_plume = OdorPlume(sim_region=sim_region,
                           max_num_puffs=200,
                           puff_init_rad=0.02,
                           odor_source=(sim_region[0]+0.02, 0., 0.))
    # odor_plume.clear_puff_nodes()
    # odor_plume.my_step()
    while odor_plume.my_step() != -1:
        odor_plume.keyboard_control()
        # pass