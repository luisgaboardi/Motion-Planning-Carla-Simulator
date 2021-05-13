# Planejamento de Movimento no Simulador Carla

## Objetivo

Desenvolver uma solução de motion planning implementada em Python utilizando o Simulador Carla.

## Requisitos/Critérios de aceitação

### Quanto ao projeto de pesquisa
- O aluno deve desenvolver uma solução (script) em Python para o problema de planejamento de veículos autônomos pelo simulador carla
- O aluno deve elaborar um artigo científico sobre planejamento de movimento e a solução desenvolvida.
- O aluno deve apresentar ambos o script e o artigo para o professor orientador para devidas correções, ajustes e eventual aprovação das atividades desenvolvidas no período de pesquisa.
- O prazo de entrega das atividades é agosto de 2021.

### Quanto à trajetória
- O veículo deve conseguir chegar de qualquer ponto A para qualquer outro ponto B;
- A trajetória e os pontos que serão percorridos devem ser selecionados e posicionados de acordo com o mapa de alta definição OpenDRIVE da região.
- A rota gerada até o destino deve ser eficiente, ou seja, sem dar voltas ou ser maior do que o necessário;
- O trajeto escolhido deve obedecer as leis de trânsito vigentes

### Quanto ao planejamento de movimento
- O veículo deve percorrer o trajeto obedecendo as leis de trânsito vigentes, tais como:
    - Velocidade máxima
    - Velocidade mínima
    - Centralização na faixa
    - Mudança de faixa
    - Direção de tráfego da via
    - Semáforos
    - Placas
    - Entre outros.

- O veículo deve percorrer o trajeto em segurança, evitando acidentes tais como:
    - Colisões com outros veículos (frontais, laterais, traseiras)
    - Colisões com placas, semáforos, cones e outros obstáculos estáticos
    - Atropelamentos de pedestres ou ciclistas

- O veículo deve percorrer o trajeto confortavelmente, de forma a permitir uma viagem agradável ao passageiro, evitando realizar:
    - Frenagens bruscas
    - Acelerações bruscas
    - Curvas bruscas
    - Curvas inconstantes
    - "Acelera, freia, acelera" típico de um trânsito congestionado
    - Entre outras ações
