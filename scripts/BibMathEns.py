# BIBLIOTECA QUE CONTEM A MATEMATICA NECESSARIA AOS ENSAIOS
import math
import numpy as np


class bib_mathEnsaio:

    __instance = None

    # Construtor da Classe
    def __init__(self):
        print('Biblioteca de calculos chamada')


        # ======== FUNCAO QUE, BASEADO NO NUMERO DE APOIOS DO ROBO, RETORNA SUA ESTRUTURA
        # INPUTS
        # 	-num_apoios: numero de apoios do robo (por exemplo, 6 caso esteja com 6 rodas)

    @staticmethod
    def instance():  # Singleton pattern
        if not bib_mathEnsaio.__instance:
            bib_mathEnsaio.__instance = bib_mathEnsaio()
        return bib_mathEnsaio.__instance

    @staticmethod
    def estruturaRobo(num_apoios):

        # Define a estrutura do robo de acordo com o numero de apoios do mesmo
        # Estas variaveis sao utilizadas pelo calculo da estabilidade
        # condicao quando esta sendo utilizado o robo com 4 rodas
        if num_apoios == 4:

            # Vetor guia para plotar o PS do robo
            vet_seq_lig = [1, 2, 4, 3]

            p_cg_f = np.matrix([[0.21156, -0.16388, -0.14176],
                                [-0.21244, -0.16386, -0.14176],
                                [0.21156, 0.16407, -0.14176],
                                [-0.21244, 0.16407, -0.14176]])

        # Condicao quando esta sendo utilizado o robo com 6 rodas
        elif num_apoios == 6:

            # Vetor guia para plotar o PS do robo
            vet_seq_lig = [1, 2, 3, 6, 5, 4]

            p_cg_f = np.matrix([[0.21156, -0.16388, -0.14176],
                                [0.00044, -0.21109, -0.14176],
                                [-0.21244, -0.16386, -0.14176],
                                [0.21156, 0.16407, -0.14176],
                                [-0.00044, 0.21127, -0.14176],
                                [-0.21244, 0.16407, -0.14176]])

        return vet_seq_lig, p_cg_f

    # ======= FUNCAO QUE CALCULA AS COORDENADAS DO ROBO COM RESPEITO AO MUNDO ============
    # INPUTS
    # 	-pos: vetor com a posicao no espaco do centro de gravidade do robo
    # 	-eul_ang: rotacao do sistema de coordenada do robo com respeito ao mundo em angulo de euler ZYX
    # 	-num_apoios: quantidade de apoios que o robo possui (por exemplo, com 6 rodas ou 4)
    # OUTPUTS
    # 	-p_mundo_robo: vetor com as coordenadas do centro do robo
    # 	-p_mundo_f:	lista de vetores com as coordenadas dos pontos de apoio do robo com respeito a ele mesmo
    def calculoCoordenadasRobo(self, pos, eul_ang, num_apoios, p_cg_f):

        # Cria o vetor para calculo de pos do robo no mundo
        p_mundo_robo = np.array([pos[0], pos[1], pos[2]]).reshape(3, 1)

        # Convertendo os angulos de euler em matriz de rotacao
        # R_mundo_robo = np.eye(3)
        R_mundo_robo = self.eulerAnglesToRotationMatrix(eul_ang)

        # Calcula a posicao dos pontos de apoio com respeito ao mundo
        p_mundo_f = []
        for i in range(0, num_apoios):
            aux = np.dot(R_mundo_robo, p_cg_f[i].transpose())
            p_mundo_f.append(p_mundo_robo + np.array(aux))

        return p_mundo_robo, p_mundo_f

    # ======= FUNCAO QUE CONVERTE VETOR DE ANGULO DE EULER EM MATRIZ DE ROTACAO
    # INPUT
    # 	-vetor com os angulos de euler (radianos, por favor)
    # OUTPUT
    # 	-Matriz de rotacao (SE3)
    @staticmethod
    def eulerAnglesToRotationMatrix(theta):

        R_x = np.array([[1, 0, 0],
                        [0, math.cos(theta[0]), -math.sin(theta[0])],
                        [0, math.sin(theta[0]), math.cos(theta[0])]
                        ])

        R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                        [0, 1, 0],
                        [-math.sin(theta[1]), 0, math.cos(theta[1])]
                        ])

        R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                        [math.sin(theta[2]), math.cos(theta[2]), 0],
                        [0, 0, 1]
                        ])

        R = np.dot(R_z, np.dot(R_y, R_x))

        return R

    # ============= FUNCAO QUE RECEBE OS PONTOS DE APOIO DO ROBO E RETORNA A ================
    # ============= MAGNITUDE DA MARGEM DE ESTABILIDADE, SEUS PONTOS INICIAL E FINAL=========
    # INPUTS
    # 	-vet_seq_lig: vetor indicando a sequencia de ligacao dos pontos para desenhar a Borda de Suporte ou Poligono
    # 	-p_apoios: coordenadas dos pontos de apoio com respeito ao mundo
    # 	-p_cg: coordenada do centro de gravidade do robo com respeito ao mundo
    # OUTPUTS
    # 	-Modulo da margem de estabilidade
    # 	-Coordenada do inicio da reta que liga o CG ao ponto mais proximo da BS
    # 	-Coordenada de fim da reta que liga o CG ao ponto mais proximo da BS
    def calculaMagMargEstabilidade(self, vet_seq_lig, p_apoios, p_centro_gravidade):

        # Extraindo a coordenada 2D do centro de gravidade do robo
        p_cg = np.array([p_centro_gravidade[0], p_centro_gravidade[1]])

        # Extraindo as coordenadas extremas de uma linha do padrao de suporte
        q1 = []
        q2 = []
        mag = []

        # Calcula a margem de estabilidade com respeito a todos os vertices do PS
        for i in range(len(p_apoios)):

            if i < len(p_apoios) - 1:
                q1.append(np.array([p_apoios[vet_seq_lig[i] - 1][0], p_apoios[vet_seq_lig[i] - 1][1]]))
                q2.append(([p_apoios[vet_seq_lig[i + 1] - 1][0], p_apoios[vet_seq_lig[i + 1] - 1][1]]))
            else:
                q1.append(([p_apoios[vet_seq_lig[i] - 1][0], p_apoios[vet_seq_lig[i] - 1][1]]))
                q2.append(([p_apoios[vet_seq_lig[0] - 1][0], p_apoios[vet_seq_lig[0] - 1][1]]))

            # Encontrando a norma da magnitude para este vertice especifico do PS
            aux_cnt = np.hstack([np.array(q2[-1]) - np.array(q1[-1]), np.array(p_cg) - np.array(q1[-1])])
            aux_num = math.fabs(np.linalg.det(aux_cnt))
            aux_den = np.linalg.norm(np.array(q2[-1]) - np.array(q1[-1]))
            mag.append(aux_num / aux_den)

    # Descobrindo a menor magnitude e seu index, que eh o valor da margem
        mag_min_index = mag.index(min(mag))

        # -- Encontrando a reta que liga cg ao vertice de menor magnitude

        # Vetor perpendicular a linha entre pontos de apoio
        v = np.array([q2[mag_min_index][1] - q1[mag_min_index][1], -(q2[mag_min_index][0] - q1[mag_min_index][0])])

        # Slope do vetor perpendicular
        slope = v[1] / v[0]

        # Encontrando o ponto intermediario - Considera-se que o x = 0
        p_cg_reta = np.array([0, (-slope * p_cg[0] + p_cg[1])])

        # linha 1
        p1 = np.array([q1[mag_min_index][0][0], q1[mag_min_index][1][0]]).T
        p2 = np.array([q2[mag_min_index][0][0], q2[mag_min_index][1][0]]).T

        # linha 2
        p3 = np.array([p_cg[0][0], p_cg[1][0]]).T
        p4 = np.array([p_cg_reta[0], p_cg_reta[1]]).T

        # Calculando a intersecao
        intersecao = self.seg_intersect(p1, p2, p3, p4)

        # Retornando os valores
        # - Valor da menor magnitude, que eh a margem
        # - Ponto inicial da reta da margem
        # - Ponto final da reta da margem

        return min(mag), np.array([p_cg[0], p_cg[1]]).T, intersecao

    # ======= FUNCOES PARA CALCULAR A INTERSECAO ENTRE DOIS PONTOS
    @staticmethod
    def perp(a):
        b = np.empty_like(a)
        b[0] = -a[1]
        b[1] = a[0]
        return b

    # line segment a given by endpoints a1, a2
    # line segment b given by endpoints b1, b2
    # return
    def seg_intersect(self, a1, a2, b1, b2):
        da = a2 - a1
        db = b2 - b1
        dp = a1 - b1
        dap = self.perp(da)
        denom = np.dot(dap, db)
        num = np.dot(dap, dp)
        return (num / denom.astype(float)) * db + b1

    # ======= FUNCAO QUE CALCULA O ANGULO DE TOMBAMENTO E A ENERGIA =========================
    # INPUTS
    # 	-vet_seq_lig: vetor indicando a sequencia de ligacao dos pontos para desenhar a Borda de Suporte ou Poligono
    # 	-p_apoios: coordenadas dos pontos de apoio com respeito ao mundo
    # 	-p_cg: coordenada do centro de gravidade do robo com respeito ao mundo
    # OUTPUTS
    # 	-Angulo de capotamento para a pose dada
    # 	-Energia de tombamento para a pose dada
    # 	-Coordenada do inicio da reta que liga o CG ao ponto mais proximo da BS
    # 	-Coordenada de fim da reta que liga o CG ao ponto mais proximo da BS
    @staticmethod
    def calculaEnergiaEstabilidade(vet_seq_lig, p_apoios, p_cg, massa, gravidade):

        # Extraindo as coordenadas extremas de uma linha do padrao de suporte
        v_R = []
        p_inter = []
        h = []
        energia = []
        angulo_cap = []
        p_cg = np.array(p_cg)
        p_apoios = np.array(p_apoios)

        np.seterr(all='ignore')

        # Calcula a margem de estabilidade com respeito a todos os vertices do PS
        for i in range(len(p_apoios)):

            # Recebendo as coordenadas dos ptos de apoio necessarios
            if i < len(p_apoios) - 1:
                f1 = p_apoios[vet_seq_lig[i] - 1]
                f2 = p_apoios[vet_seq_lig[i + 1] - 1]
            else:
                f1 = p_apoios[vet_seq_lig[i] - 1]
                f2 = p_apoios[vet_seq_lig[0] - 1]

            # Vetores ligando f1 a f2 e f1 ao cg
            # v_f1f2 = (f2 - f1).T[0]
            v_f1f2 = (f2 - f1).T
            # v_f1cg = (p_cg - f1).T[0]
            v_f1cg = (p_cg - f1).T

            # Vetor projecao de vf1cg em vf1f2
            v_proj_vf1cg_vf1f2 = ((np.dot(v_f1cg, v_f1f2)) / (np.linalg.norm(v_f1f2) ** 2)) * v_f1f2

            # Ponto final da projecao (que eh a intersecao das duas linhas)
            # p_inter.append(f1.T[0] + v_proj_vf1cg_vf1f2)
            p_inter.append(f1.T + v_proj_vf1cg_vf1f2)

            # Vetor ligando cg a intersecao
            # v_R.append((p_cg.T - p_inter[-1].T)[0])
            v_R.append(p_cg.T - p_inter[-1].T)

            # Vetor normal ao plano unitario
            z = np.array([0, 0, 1])
            v_NPlano = np.cross(z, v_f1f2)
            v_NPlano = v_NPlano / np.linalg.norm(v_NPlano)

            # Projecao de v_R no plano para encontrar v_Rlinha
            v_Rlinha = v_R[-1] - (((np.dot(v_R[-1], v_NPlano)) / np.linalg.norm(v_NPlano) ** 2) * v_NPlano)

            # Angulo entre v_R e v_Rlinha
            theta_vR_vRlinha = np.arctan2(np.linalg.norm(np.cross(v_R[-1], v_Rlinha)), np.dot(v_R[-1], v_Rlinha))

            # Angulo entre v_Rlinha e z_chapeu
            psi_vRlinha_zchapeu = np.arctan2(np.linalg.norm(np.cross(v_Rlinha, z)), np.dot(v_Rlinha, z))

            # Finalmente, calculo altura de deslocamento do cg
            h.append(np.linalg.norm(v_R[-1]) * (1 - math.cos(theta_vR_vRlinha)) * (math.cos(psi_vRlinha_zchapeu)))

            # Calculo da energia potencial para o h encontrado
            # Eh feito o calculo pelo absoluto para evitar problemas com cosseno negativo
            energia.append(massa * gravidade * math.fabs(h[-1]))

            # Angulo de capotamento
            # Este eh o angulo entre -v_R e -z_chapeu
            angulo_cap.append(
                180 - math.degrees(math.atan2(np.linalg.norm(np.cross(v_R[-1], -z)), np.dot(v_R[-1], -z))))

        # Descobrindo o index da menor energia
        energia_min_index = energia.index(min(energia))

        return min(angulo_cap), min(energia), p_cg, p_inter[energia_min_index]

    # ======== FUNCAO QUE REALIZA OS CALCULOS ENERGETICOS DO ENSAIO =========================
    # INPUTS
    # 	-vel_linear: vetor com a velocidade linear em cada eixo do centro do robo
    # 	-torque: vetor de 6 pos com os torques aplicados por motor
    # 	-vel: vetor de 6 pos com a velocidade aplicada por motor
    # OUTPUTS
    # 	-vel_tranl: modulo de velocidade tranlacional do centro do robo (m/s)
    # 	-resistencia_esp_instantanea_robo: resistencia especifica instantanea calculada
    # 	-maxima_tensao_bateria: Maxima tensao retirada da bateria
    # 	-aux_potencia_instantanea_pmotor: vetor com P instantanea retirada por motor
    # 	-aux_potencia_instantanea_total: soma das potencias gastas por motor
    # 	-aux_corrente_instantanea_robo: corrente consumida no total pelo robo
    @staticmethod
    def calculaEnergetico(vel_linear, torque, vel, reducao_eficiencia, reducao_proporcao, massa,
                      gravidade, const_velocidade, const_torque):

        # Variaveis auxiliares necessarias
        aux_potencia_instantanea_pmotor = []
        aux_tensao_instantanea_pmotor = []
        aux_potencia_instantanea_total = 0
        aux_corrente_instantanea_robo = 0

        # Calcula a velocidade de translacao no espaco do robo
        # Converte as velocidades transl nos eixos na total do corpo
        vel_transl = (vel_linear[0] ** 2 + vel_linear[1] ** 2 + vel_linear[2] ** 2) ** 0.5

        for i in range(0, 6):
            # Torque desenvolvido no motor, antes da reducao e considerando a eficiencia desta
            Td = (torque[i] / reducao_proporcao) * (2 - reducao_eficiencia)

            # Velocidade desenvolvido no motor antes da reducao
            Vd = vel[i] * reducao_proporcao

            # Calculo da energia gasta por motor
            aux_potencia_instantanea_pmotor.append(
                (math.fabs(Td) / const_torque) * (math.fabs(Vd) / const_velocidade))

            # Somatorio da energia toral (em Watts)
            aux_potencia_instantanea_total += math.fabs(aux_potencia_instantanea_pmotor[-1])

            # Corrente instantanea retirada da bateria (em Amperes)
            aux_corrente_instantanea_robo += math.fabs(Td) / const_torque

            # Tensao instantanea retirada da bateria
            aux_tensao_instantanea_pmotor.append(math.fabs(Vd) / const_velocidade)

        # Calculo da resistencia especifica instantanea do robo
        resistencia_esp_instantanea_robo = aux_potencia_instantanea_total/(massa * gravidade * vel_transl)

        # Maxima tensao retirada da bateria neste instante de tempo
        maxima_tensao_bateria = max(aux_tensao_instantanea_pmotor)

        # Salvando valores calculados em lista
        potencia_instantanea_robo = aux_potencia_instantanea_total
        corrente_instantanea_robo = aux_corrente_instantanea_robo
        potencia_instantanea_pmotor = aux_potencia_instantanea_pmotor

        # Retornar
        # vel_transl
        return vel_transl, resistencia_esp_instantanea_robo, maxima_tensao_bateria, aux_potencia_instantanea_pmotor, \
               aux_potencia_instantanea_total, aux_corrente_instantanea_robo
