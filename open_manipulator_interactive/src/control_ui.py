#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

import flet as ft

from open_manipulator_interactive import Director



def main_page(page: ft.Page):

    global director

    page.title = "Taller: Interaccion con Brazos Roboticos"
    page.vertical_alignment = ft.MainAxisAlignment.CENTER
    page.horizontal_alignment = ft.CrossAxisAlignment.CENTER

    def task_space_click(key):
        director.send_goal_task_position(key)


    def tool_click(key):
        director.send_tool_joint_position(key)


    def container_clickable_position(text,key,color):
        return ft.Container(
                    content=ft.Text(text),
                    margin=10,
                    padding=10,
                    alignment=ft.alignment.center,
                    bgcolor=color,
                    width=150,
                    height=150,
                    border_radius=10,
                    on_click=lambda e: task_space_click(key), 
                )
    

    def container_clickable_gripper(text,key,color):
        return ft.Container(
                    content=ft.Text(text),
                    margin=10,
                    padding=10,
                    alignment=ft.alignment.center,
                    bgcolor=color,
                    width=150,
                    height=150,
                    border_radius=10,
                    on_click=lambda e: tool_click(key), 
                )


    def position_pad():
        return ft.Column(
            [
                ft.Row(
                    [
                        container_clickable_position("H1","high_1",ft.colors.LIGHT_BLUE_100),
                        container_clickable_position("H2","high_2",ft.colors.LIGHT_BLUE_100),
                        container_clickable_position("H3","high_3",ft.colors.LIGHT_BLUE_100),
                    ],
                    alignment=ft.MainAxisAlignment.CENTER
                    ),
                
                ft.Row(
                    [
                        container_clickable_position("M1","mid_1",ft.colors.LIGHT_BLUE_300),
                        container_clickable_position("M2","mid_2",ft.colors.LIGHT_BLUE_300),
                        container_clickable_position("M3","mid_3",ft.colors.LIGHT_BLUE_300),
                    ],
                    alignment=ft.MainAxisAlignment.CENTER
                    ),
                
                ft.Row(
                    [
                        container_clickable_position("L1","low_1",ft.colors.LIGHT_BLUE_600),
                        container_clickable_position("L2","low_2",ft.colors.LIGHT_BLUE_600),
                        container_clickable_position("L3","low_3",ft.colors.LIGHT_BLUE_600),
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
                        container_clickable_gripper("OPEN",True,ft.colors.LIGHT_GREEN_200),
                        container_clickable_gripper("CLOSE",False,ft.colors.RED_200)
                    ],
                    alignment=ft.MainAxisAlignment.CENTER,
                )
            ],
            alignment=ft.MainAxisAlignment.CENTER,
            horizontal_alignment=ft.CrossAxisAlignment.CENTER
        )

    page.add(
        ft.Row(
            [
                position_pad(),
                gripper_pad()
            ],
            alignment=ft.MainAxisAlignment.CENTER
            )
    )


def main():
    global director
    director = Director()
    
    ft.app(target=main_page)

if __name__ == "__main__":
    main()
