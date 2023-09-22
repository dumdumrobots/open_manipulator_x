#!/usr/bin/env python3

import rospy
import flet as ft
from open_manipulator_interactive import Director


def main_page(page: ft.Page):

    global director

    page.title = "Taller: Interacción con Brazos Robóticos"
    page.vertical_alignment = ft.MainAxisAlignment.CENTER
    page.horizontal_alignment = ft.CrossAxisAlignment.CENTER

    lamp_movement_icon = ft.Ref[ft.Icon]()
    lamp_actuator_icon = ft.Ref[ft.Icon]()

    lamp_movement_text = ft.Ref[ft.Text]()
    lamp_actuator_text = ft.Ref[ft.Text]()

    def task_space_click(key):
        director.send_goal_task_position(key)


    def tool_click(key):
        director.send_tool_joint_position(key)


    def container_clickable_position(text,key):
        return ft.Stack(
            [
                ft.Container(
                    content=ft.Text(text,size=30,color=ft.colors.WHITE),
                    margin=10,
                    padding=10,
                    alignment=ft.alignment.center,
                    bgcolor=ft.colors.LIGHT_BLUE_400,
                    width=150,
                    height=150,
                    border_radius=10,
                ),
                ft.Column(
                    [
                        ft.Icon(name=ft.icons.CIRCLE_OUTLINED, 
                        color=ft.colors.WHITE,
                        size=100)

                    ],
                    left=35,
                    top=35
                ),
                ft.Container(
                    alignment=ft.alignment.center,
                    bgcolor=ft.colors.TRANSPARENT,
                    width=150,
                    height=150,
                    border_radius=10,
                    on_click=lambda e: task_space_click(key), 
                    ink=True,
                    left=10,
                    top=10
                ),
            ]
        )
    

    def container_clickable_gripper(text,key,color,text_color):
        return ft.Container(
                    content=ft.Text(text,size=30,color=text_color),
                    margin=10,
                    padding=10,
                    alignment=ft.alignment.center,
                    bgcolor=color,
                    width=150,
                    height=150,
                    border_radius=10,
                    on_click=lambda e: tool_click(key),
                    ink=True,
                )


    def robot_moving_lamp():
        return ft.Column(
            [
                ft.Container(
                    content=ft.Icon(ref=lamp_movement_icon,name=ft.icons.CIRCLE_ROUNDED,color=ft.colors.RED_500,size=100),
                    alignment=ft.alignment.center,
                    padding=10,
                    width=150,
                    height=115,
                    border_radius=10
                    ),
                ft.Container(
                    content=ft.Text(ref=lamp_movement_text,value="Robot",size=15,text_align=ft.TextAlign.CENTER),
                    alignment=ft.alignment.center,
                    margin=10,
                    padding=10,
                    bgcolor=ft.colors.LIGHT_BLUE_100,
                    width=150,
                    height=50,
                    border_radius=10
                    ),
            ],
            alignment=ft.MainAxisAlignment.CENTER,
            horizontal_alignment=ft.CrossAxisAlignment.CENTER
        )
    
    def actuator_enabled_lamp():
        return ft.Column(
            [
                ft.Container(
                    content=ft.Icon(ref=lamp_actuator_icon,name=ft.icons.CIRCLE_ROUNDED,color=ft.colors.RED_500,size=100),
                    alignment=ft.alignment.center,
                    padding=10,
                    width=150,
                    height=115,
                    border_radius=10
                    ),
                ft.Container(
                    content=ft.Text(ref=lamp_actuator_text,value="Robot",size=15,text_align=ft.TextAlign.CENTER),
                    alignment=ft.alignment.center,
                    margin=10,
                    padding=10,
                    bgcolor=ft.colors.LIGHT_BLUE_100,
                    width=150,
                    height=50,
                    border_radius=10
                    ),
            ],
            alignment=ft.MainAxisAlignment.CENTER,
            horizontal_alignment=ft.CrossAxisAlignment.CENTER
        )
    
    def position_pad():
        return ft.Column(
            [
                ft.Row(
                    [
                        container_clickable_position("H1","high_1"),
                        container_clickable_position("H2","high_2"),
                        container_clickable_position("H3","high_3"),
                    ],
                    alignment=ft.MainAxisAlignment.CENTER
                    ),
                
                ft.Row(
                    [
                        container_clickable_position("M1","mid_1"),
                        container_clickable_position("M2","mid_2"),
                        container_clickable_position("M3","mid_3"),
                    ],
                    alignment=ft.MainAxisAlignment.CENTER
                    ),
                
                ft.Row(
                    [
                        container_clickable_position("L1","low_1"),
                        container_clickable_position("L2","low_2"),
                        container_clickable_position("L3","low_3"),
                    ],
                    alignment=ft.MainAxisAlignment.CENTER
                    )
                
            ],
            alignment=ft.MainAxisAlignment.CENTER,
            horizontal_alignment=ft.CrossAxisAlignment.CENTER
            )
    

    def gripper_pad():
        return ft.Column(
            [
                ft.Row(
                    [
                        robot_moving_lamp(),
                        actuator_enabled_lamp(),
                    ],
                    alignment=ft.MainAxisAlignment.CENTER

                ),

                ft.Row(
                    [
                        container_clickable_gripper("ABRIR",True,ft.colors.LIGHT_GREEN_200,ft.colors.LIGHT_GREEN_900),
                        container_clickable_gripper("CERRAR",False,ft.colors.RED_200,ft.colors.RED_900)
                    ],
                    alignment=ft.MainAxisAlignment.CENTER,
                ),
            ],
            alignment=ft.MainAxisAlignment.CENTER,
            horizontal_alignment=ft.CrossAxisAlignment.CENTER
        )
    

    def robot_ui():
        return ft.Column(
            [
                ft.Row(
                    [
                        gripper_pad(),
                        position_pad()
                
                    ],
                    spacing=100,
                    alignment=ft.MainAxisAlignment.CENTER
                )
            ],
        )


    def lamp_update():
        if director.robot_moving:
            lamp_movement_icon.current.color = ft.colors.LIGHT_GREEN_500
            lamp_movement_text.current.value = "En movimiento."
        else:
            lamp_movement_icon.current.color = ft.colors.RED_500
            lamp_movement_text.current.value = "Robot detenido."

        if director.actuator_enable:
            lamp_actuator_icon.current.color = ft.colors.LIGHT_GREEN_500
            lamp_actuator_text.current.value = "Robot habilitado."
        else:
            lamp_actuator_icon.current.color = ft.colors.RED_500
            lamp_actuator_text.current.value = "Robot deshabilitado."

    page.add(robot_ui())

    while not rospy.is_shutdown():
        lamp_update()
        
        page.update()

        director.rate.sleep()
    


def main():

    global director
    director = Director()
    
    ft.app(target=main_page)

if __name__ == "__main__":
    main()
