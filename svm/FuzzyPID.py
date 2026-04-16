import numpy as np
import time
import serial  # 示例集成
import os

class Membership:
    @staticmethod
    def triangle(x, a, b, c):
        return np.maximum(np.minimum((x - a) / (b - a + 1e-6), (c - x) / (c - b + 1e-6)), 0)

class FuzzyPID:
    def __init__(self, table_path='svm/fuzzy_table.npy'):
        self.Kp_base = 0.1
        self.Ki_base = 0.01  # 加积分
        self.Kd_base = 0.2
        self.Kp = self.Kp_base
        self.Kd = self.Kd_base
        self.table_path = table_path
        self.load_or_create_table()
        self.anfis = ANFISDeltaK(n_rules=9)
    
    def create_table(self):
        # 3级: N/Z/P
        labels = ['N', 'Z', 'P']
        e_centers = [-30, 0, 30]  # e -50~50
        de_centers = [-12, 0, 12]  # de -20~20
        e_grid = np.linspace(-50, 50, 101)
        de_grid = np.linspace(-20, 20, 41)
        
        # 规则表 (9规则, 经验: 大|e|增Kp, 大|de|增Kd)
        rules = {
            ('N','N'): ('P','P'), ('N','Z'): ('P','Z'), ('N','P'): ('Z','P'),
            ('Z','N'): ('Z','P'), ('Z','Z'): ('Z','Z'), ('Z','P'): ('Z','P'),
            ('P','N'): ('P','P'), ('P','Z'): ('P','Z'), ('P','P'): ('Z','P'),
        }
        kp_out = {'N': -0.01, 'Z': 0, 'P': 0.01}
        kd_out = {'N': -0.05, 'Z': 0, 'P': 0.05}
        
        E, DE = np.meshgrid(e_grid, de_grid)
        table_kp = np.zeros_like(E)
        table_kd = np.zeros_like(E)
        
        memb = Membership()
        for i_de, de in enumerate(de_grid):
            for i_e, e in enumerate(e_grid):
                mu_e = {l: memb.triangle(e, c-20, c, c+20) for l,c in zip(labels, e_centers)}
                mu_de = {l: memb.triangle(de, c-8, c, c+8) for l,c in zip(labels, de_centers)}
                
                agg_kp, agg_kd, sum_mu = 0, 0, 0
                for (el, del_), (kp_l, kd_l) in rules.items():
                    mu = min(mu_e[el], mu_de[del_])
                    agg_kp += mu * kp_out[kp_l]
                    agg_kd += mu * kd_out[kd_l]
                    sum_mu += mu
                table_kp[i_de, i_e] = agg_kp / (sum_mu + 1e-6)
                table_kd[i_de, i_e] = agg_kd / (sum_mu + 1e-6)
        
        self.table = np.stack([table_kp, table_kd])  # [2,41,101]
        np.save(self.table_path, self.table)
        print('Table created & saved.')
    
    def load_or_create_table(self):
        if os.path.exists(self.table_path):
            self.table = np.load(self.table_path)
        else:
            self.create_table()
    
    def infer_fast(self, e, de):
        i_e = np.argmin(np.abs(e_grid - e))  # e_grid/de_grid global or self
        i_de = np.argmin(np.abs(de_grid - de))
        dkp = self.table[0, i_de, i_e]
        dkd = self.table[1, i_de, i_e]
        return dkp, dkd
    
    def update(self, e, de, dt=0.033):
        dkp_fuzzy, dkd_fuzzy = self.infer_fast(e, de)  # 教师信号
        dkp, dkd, w_bar = self.anfis.forward(e, de)
        loss = self.anfis.backward(e, de, dkp_fuzzy, dkd_fuzzy, w_bar)  # 模仿模糊
        self.Kp = np.clip(self.Kp_base + dkp, 0.01, 0.3)
        self.Kd = np.clip(self.Kd_base + dkd, 0.05, 0.5)
        self.integral_e = getattr(self, 'integral_e', 0) + e * dt  # trapz
        self.integral_e = np.clip(self.integral_e, -100,100)
        u_raw = self.Kp * e + self.Ki_base * self.integral_e + self.Kd * de
        self.u_smooth = 0.8 * getattr(self, 'u_smooth', 0) + 0.2 * u_raw
        u = self.u_smooth
        return u, self.Kp, self.Kd, loss

class ANFISDeltaK:  # 输出ΔKp, ΔKd (2输出)
    def __init__(self, n_rules=9, n_memb=3):
        self.n_rules = n_rules
        self.n_memb = n_memb
        self.c_e = np.linspace(-20,20,n_memb)  # 匹配step amp
        self.c_de = np.linspace(-20,20,n_memb)
        self.sigma_e = np.ones(n_memb)*10
        self.sigma_de = np.ones(n_memb)*5
        self.p_kp = np.random.uniform(-0.01,0.01,(n_rules,1))  # ΔKp params
        self.q_kp = np.random.uniform(-0.01,0.01,(n_rules,1))
        self.r_kp = np.zeros((n_rules,1))
        self.p_kd = np.random.uniform(-0.05,0.05,(n_rules,1))  # ΔKd
        self.q_kd = np.random.uniform(-0.05,0.05,(n_rules,1))
        self.r_kd = np.zeros((n_rules,1))
        self.lr = 0.0005  # 更小
        self.l2_lambda = 0.001

    def gaussian_memb(self, x, c, sigma):
        return np.exp(-0.5 * ((x - c)/sigma)**2)

    def forward(self, e, de):
        mu_e = np.array([self.gaussian_memb(e, self.c_e[i], self.sigma_e[i]) for i in range(self.n_memb)])
        mu_de = np.array([self.gaussian_memb(de, self.c_de[i], self.sigma_de[i]) for i in range(self.n_memb)])
        w = np.outer(mu_e, mu_de).flatten()[:self.n_rules]
        w_bar = w / (np.sum(w) + 1e-8)
        f_kp = np.clip(self.p_kp*e + self.q_kp*de + self.r_kp, -0.05,0.05)
        f_kd = np.clip(self.p_kd*e + self.q_kd*de + self.r_kd, -0.1,0.1)
        dkp = np.sum(w_bar[:,np.newaxis] * f_kp)
        dkd = np.sum(w_bar[:,np.newaxis] * f_kd)
        return dkp, dkd, w_bar

    def backward(self, e, de, dkp_des, dkd_des, w_bar):
        dkp, dkd, _ = self.forward(e, de)
        err_kp = dkp - dkp_des
        err_kd = dkd - dkd_des
        # ΔKp backward
        delta_p_kp = np.clip(w_bar * err_kp * e, -0.01,0.01)
        delta_q_kp = np.clip(w_bar * err_kp * de, -0.01,0.01)
        delta_r_kp = np.clip(w_bar * err_kp, -0.01,0.01)
        self.p_kp -= self.lr * (delta_p_kp[:,np.newaxis] + 2*self.l2_lambda*self.p_kp)
        self.q_kp -= self.lr * (delta_q_kp[:,np.newaxis] + 2*self.l2_lambda*self.q_kp)
        self.r_kp -= self.lr * (delta_r_kp[:,np.newaxis] + 2*self.l2_lambda*self.r_kp)
        # ΔKd similar
        delta_p_kd = np.clip(w_bar * err_kd * e, -0.01,0.01)
        delta_q_kd = np.clip(w_bar * err_kd * de, -0.01,0.01)
        delta_r_kd = np.clip(w_bar * err_kd, -0.01,0.01)
        self.p_kd -= self.lr * (delta_p_kd[:,np.newaxis] + 2*self.l2_lambda*self.p_kd)
        self.q_kd -= self.lr * (delta_q_kd[:,np.newaxis] + 2*self.l2_lambda*self.q_kd)
        self.r_kd -= self.lr * (delta_r_kd[:,np.newaxis] + 2*self.l2_lambda*self.r_kd)
        return float(np.mean(np.abs([err_kp, err_kd])))

# Global grids (共享)
e_grid = np.linspace(-50,50,101)
de_grid = np.linspace(-20,20,41)

if __name__ == '__main__':
    fpid = FuzzyPID()
    print('KP/KD base:', fpid.Kp, fpid.Kd)
    
    # 实时测试
    times, es, des, us, kps, losses = [], [], [], [], [], []
    t0 = time.time()
    x = 0
    v = 0
    dt = 0.01
    tau = 0.3
    target = 0
    step_time = 500
    for i in range(3000):  # 延长
        if i == step_time:
            target = 20
        e = target - x
        de = -v
        u, kp, kd, loss = fpid.update(e, de)
        v += dt * (u / tau - 0.1*v + np.random.normal(0,0.3))  # 减噪
        x += dt * v
        times.append(time.time()-t0)
        es.append(e); des.append(de); us.append(u); kps.append(kp)
        losses.append(loss)
    print(f'Mean |e| after step: {np.mean(np.abs(es[step_time:])):.4f}')
    print(f'Final loss: {losses[-1]:.4f}, Final e: {es[-1]:.4f}')
    
    print(f'Mean infer time: {np.mean(np.diff(times))*1000:.2f} ms')
    import matplotlib.pyplot as plt
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

    ax1.plot(es, 'b', label='Error e')
    ax1_u = ax1.twinx()
    ax1_u.plot(us, 'r', alpha=0.7, label='Control u')
    ax1.axvline(step_time, color='k', ls='--', label='Step')
    ax1.set_title('Closed-loop Step Response')
    ax1.legend(loc='upper left'); ax1_u.legend(loc='upper right')

    ax2.plot(kps, 'g', label='Kp Adaptation')
    ax2.set_title('Controller Gain Adaptation')
    ax2.legend()
    plt.tight_layout(); plt.show()
    print(f'Steady |e| mean: {np.mean(np.abs(es[step_time+200:])):.4f}')